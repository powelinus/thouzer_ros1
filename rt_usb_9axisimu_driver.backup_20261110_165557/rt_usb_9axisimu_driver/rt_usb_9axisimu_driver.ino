#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// Register MPU6500 / MPU9250
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_CFG2 0x1D
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_PWR_MGMT_2 0x6C
#define REG_WHO_AM_I 0x75

// RT-USB binary packet
static const uint8_t PACKET_SIZE = 28;
static uint8_t timestamp8 = 0;

static uint32_t nextSampleUs = 0;
static const uint32_t SAMPLE_PERIOD_US = 20000;  // 50 Hz

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *data, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(
    (uint8_t)MPU_ADDR, (uint8_t)len, (uint8_t) true);

  if (received != len) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }

  return true;
}

int16_t readBE16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void putLE16(uint8_t *packet, uint8_t offset, int16_t value) {
  uint16_t v = (uint16_t)value;
  packet[offset] = (uint8_t)(v & 0xFF);
  packet[offset + 1] = (uint8_t)(v >> 8);
}

bool initMPU() {
  uint8_t who = 0;

  if (!readRegs(REG_WHO_AM_I, &who, 1)) {
    return false;
  }

  // MPU6500 = 0x70, MPU9250 = 0x71
  if (who != 0x70 && who != 0x71 && who != 0x73) {
    return false;
  }

  if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;
  delay(100);

  if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;
  delay(20);

  if (!writeReg(REG_PWR_MGMT_2, 0x00)) return false;

  // DLPF enabled
  if (!writeReg(REG_CONFIG, 0x03)) return false;

  // Gyro output 1 kHz / (1 + 19) = 50 Hz
  if (!writeReg(REG_SMPLRT_DIV, 19)) return false;

  // Gyroscope +/-250 dps: 131 LSB/(deg/s)
  if (!writeReg(REG_GYRO_CONFIG, 0x18)) return false;

  // Accelerometer +/-2g: 16384 LSB/g
  if (!writeReg(REG_ACCEL_CONFIG, 0x18)) return false;

  // Accelerometer DLPF
  if (!writeReg(REG_ACCEL_CFG2, 0x03)) return false;

  delay(50);
  return true;
}

bool readMPU(
  int16_t &ax, int16_t &ay, int16_t &az,
  int16_t &temp,
  int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t data[14];

  if (!readRegs(REG_ACCEL_XOUT_H, data, sizeof(data))) {
    return false;
  }

  ax = readBE16(&data[0]);
  ay = readBE16(&data[2]);
  az = readBE16(&data[4]);
  temp = readBE16(&data[6]);
  gx = readBE16(&data[8]);
  gy = readBE16(&data[10]);
  gz = readBE16(&data[12]);

  return true;
}

void sendRTPacket(
  int16_t ax, int16_t ay, int16_t az,
  int16_t temp,
  int16_t gx, int16_t gy, int16_t gz) {
  uint8_t packet[PACKET_SIZE] = { 0 };

  // Header RT-USB
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 'R';
  packet[3] = 'T';

  // Device ID
  packet[4] = 0x00;
  packet[5] = 0x00;

  // Firmware version, timestamp
  packet[6] = 5;
  packet[7] = timestamp8++;

  // Accelerometer
  putLE16(packet, 8, ax);
  putLE16(packet, 10, ay);
  putLE16(packet, 12, az);

  // Temperature
  putLE16(packet, 14, temp);

  // Gyroscope
  putLE16(packet, 16, gx);
  putLE16(packet, 18, gy);
  putLE16(packet, 20, gz);

  // Magnetometer unavailable: placeholder
  putLE16(packet, 22, 0);
  putLE16(packet, 24, 0);
  putLE16(packet, 26, 0);

  // Exactly 28 bytes; no println, no debug text
  Serial.write(packet, sizeof(packet));
}

int32_t gyroOffsetX = 0;
int32_t gyroOffsetY = 0;
int32_t gyroOffsetZ = 0;

void calibrateGyro() {
  int64_t sumX = 0;
  int64_t sumY = 0;
  int64_t sumZ = 0;

  const int samples = 500;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, temp;
    int16_t gx, gy, gz;

    if (readMPU(ax, ay, az, temp, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      validSamples++;
    }

    delay(5);
  }

  if (validSamples > 0) {
    gyroOffsetX = sumX / validSamples;
    gyroOffsetY = sumY / validSamples;
    gyroOffsetZ = sumZ / validSamples;
  }
}

void setup() {
  // Native USB CDC
  Serial.begin(57600);

  Wire.begin();
  Wire.setClock(100000);

  // Keep the port clean: do not print errors to Serial.
  while (!initMPU()) {
    delay(1000);
  }
  calibrateGyro();
  nextSampleUs = micros() + SAMPLE_PERIOD_US;
}

void loop() {
  // Avoid sending data until USB CDC is connected.
  if (!Serial) {
    timestamp8 = 0;
    nextSampleUs = micros() + SAMPLE_PERIOD_US;
    delay(10);
    return;
  }

  uint32_t now = micros();

  if ((int32_t)(now - nextSampleUs) < 0) {
    delay(1);
    return;
  }

  nextSampleUs = now + SAMPLE_PERIOD_US;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;

  if (!readMPU(ax, ay, az, temp, gx, gy, gz)) {
    return;
  }
  gx = (int16_t)(gx - gyroOffsetX);
  gy = (int16_t)(gy - gyroOffsetY);
  gz = (int16_t)(gz - gyroOffsetZ);
  // Avoid intentionally building up a backlog of packets.
  if (Serial.availableForWrite() >= PACKET_SIZE) {
    sendRTPacket(ax, ay, az, temp, gx, gy, gz);
  }
}