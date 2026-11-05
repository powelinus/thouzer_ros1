// ESP32 + MPU9250 -> Wi-Fi UDP -> ROS 1 bridge
// Example for a classic ESP32 DevKit: SDA=21, SCL=22, sensor supply=3.3V.
// The ESP32 and ROS PC must be connected to the same reachable LAN.
// Change WIFI_SSID, WIFI_PASSWORD and LAPTOP_IP before flashing.
// No magnetometer/orientation: this sketch sends 6-axis RAW data only.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char *WIFI_SSID = "RMS-10X-AAZ03";
const char *WIFI_PASSWORD = "81392063";
IPAddress LAPTOP_IP(192, 168, 212, 101);  // Ganti dengan IP laptop pada jaringan ESP32 192.168.212.101/24
const uint16_t UDP_DEST_PORT = 44444;
const uint8_t MPU_ADDR = 0x68;            // AD0=GND; use 0x69 if AD0=3V3
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const float G0 = 9.80665f;
const float D2R = 0.017453292519943295f;
const float ACCEL_LSB_PER_G = 16384.0f;   // +/- 2 g
const float GYRO_LSB_PER_DPS = 131.0f;    // +/- 250 deg/s

WiFiUDP udp;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
uint32_t sequenceNo = 0;
uint32_t nextSample = 0;

struct RawImu {
  int16_t ax, ay, az, gx, gy, gz;
};

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBytes(uint8_t startReg, uint8_t *dst, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true) != len) return false;
  for (uint8_t i = 0; i < len; ++i) dst[i] = Wire.read();
  return true;
}

uint8_t readReg(uint8_t reg) {
  uint8_t value = 0;
  if (!readBytes(reg, &value, 1)) return 0;
  return value;
}

int16_t be16(const uint8_t *data) {
  return (int16_t)((((uint16_t)data[0]) << 8) | data[1]);
}

bool readImu(RawImu &v) {
  uint8_t buf[14];
  if (!readBytes(0x3B, buf, 14)) return false;
  v.ax = be16(buf + 0);
  v.ay = be16(buf + 2);
  v.az = be16(buf + 4);
  v.gx = be16(buf + 8);
  v.gy = be16(buf + 10);
  v.gz = be16(buf + 12);
  return true;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t who = readReg(0x75); // WHO_AM_I: MPU9250 typically 0x71
  Serial.printf("WHO_AM_I = 0x%02X\n", who);
  if (who != 0x71 && who != 0x73) {
    Serial.println("MPU9250/MPU9255 not detected. Check wiring, voltage and AD0.");
    while (true) delay(1000);
  }
  bool ok = true;
  ok &= writeReg(0x6B, 0x01); // Wake, PLL clock
  ok &= writeReg(0x6C, 0x00); // Enable axes
  ok &= writeReg(0x1A, 0x03); // Gyro low pass filter ~41Hz
  ok &= writeReg(0x19, 0x09); // 1 kHz / (1+9) = 100Hz internal sample
  ok &= writeReg(0x1B, 0x00); // Gyro +/- 250 dps
  ok &= writeReg(0x1C, 0x00); // Accel +/- 2g
  ok &= writeReg(0x1D, 0x03); // Accel low pass filter
  if (!ok) {
    Serial.println("Sensor configuration failed.");
    while (true) delay(1000);
  }
  delay(150);

  Serial.println("KEEP THE ROBOT STILL: calibrating gyro bias (about 3 s).");
  double sx = 0, sy = 0, sz = 0;
  int count = 0;
  for (int i = 0; i < 300; ++i) {
    RawImu r;
    if (readImu(r)) { sx += r.gx; sy += r.gy; sz += r.gz; count++; }
    delay(10);
  }
  if (count < 250) {
    Serial.println("Too many IMU read errors.");
    while (true) delay(1000);
  }
  gyroBiasX = (float)(sx / count);
  gyroBiasY = (float)(sy / count);
  gyroBiasZ = (float)(sz / count);
  Serial.printf("Gyro raw offsets: %.2f %.2f %.2f\n", gyroBiasX, gyroBiasY, gyroBiasZ);

  connectWiFi();
  udp.begin(44445); // Local UDP source port, separate from destination
  nextSample = millis();
  Serial.println("Streaming seq,ax,ay,az,gx,gy,gz in SI units over UDP.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if ((int32_t)(millis() - nextSample) < 0) return;
  nextSample += 20;  // 50 Hz
  if ((int32_t)(millis() - nextSample) > 20) nextSample = millis() + 20;

  RawImu r;
  if (!readImu(r)) return;
  float ax = ((float)r.ax / ACCEL_LSB_PER_G) * G0;
  float ay = ((float)r.ay / ACCEL_LSB_PER_G) * G0;
  float az = ((float)r.az / ACCEL_LSB_PER_G) * G0;
  float gx = ((float)r.gx - gyroBiasX) / GYRO_LSB_PER_DPS * D2R;
  float gy = ((float)r.gy - gyroBiasY) / GYRO_LSB_PER_DPS * D2R;
  float gz = ((float)r.gz - gyroBiasZ) / GYRO_LSB_PER_DPS * D2R;

  char data[160];
  snprintf(data, sizeof(data), "%lu,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f",
           (unsigned long)sequenceNo++, ax, ay, az, gx, gy, gz);
  udp.beginPacket(LAPTOP_IP, UDP_DEST_PORT);
  udp.write((const uint8_t*)data, strlen(data));
  udp.endPacket();
}
