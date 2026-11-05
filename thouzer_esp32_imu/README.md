# THOUZER: ESP32 + MPU-9250 -> ROS 1 Melodic Docker (MQTT unchanged)

This example replaces **the ROS IMU data feed**, not the DOOG robot controller. It does **not** replace encoders, LiDAR, motor safety or an absolute pose estimate. No LiDAR connection is opened. Use only on a trusted, isolated Wi-Fi network.

## Wiring (classic ESP32 DevKit)

| ESP32 | MPU-9250 breakout |
|---|---|
| 3V3 | VCC (verify module requires 3.3 V) |
| GND | GND |
| GPIO 21 | SDA |
| GPIO 22 | SCL |
| GND | AD0 (optional; address 0x68) |

Some breakouts need 4.7 kOhm pull-ups on SDA/SCL to 3.3 V. Confirm that I2C signal levels never exceed 3.3 V. Never connect sensor directly to the THOUZER motor power rail.

**Physical installation:** mount ESP32 and IMU rigidly on the moving THOUZER chassis. Align IMU **X forward, Y left, Z up** initially; protect from vibration and keep away from motors and power cables. If you cannot align the axes, transform the sensor data correctly before publishing (the stock `odom.py` assumes Z is yaw). Power ESP32 from a regulated USB 5 V supply or power bank on the robot. If the laptop rides on the robot, USB serial is an alternative; this example uses Wi-Fi.

## Network

1. Connect ESP32 and laptop to the same reachable Wi-Fi network (e.g. laptop hotspot); keep THOUZER MQTT LAN on the laptop's other interface.
2. Find the laptop's Wi-Fi IPv4 using `ip -4 addr` (do not copy the sample IP blindly).
3. Set `WIFI_SSID`, `WIFI_PASSWORD`, and `LAPTOP_IP` in `esp32_mpu9250_udp/esp32_mpu9250_udp.ino`.
4. Program ESP32 using Arduino IDE (board: ESP32 Dev Module). Only built-in Wire, WiFi and WiFiUdp libraries are used. Use Serial Monitor at 115200. Keep THOUZER stationary during 3-second gyro calibration at boot. The sketch streams 50 Hz UDP to laptop port 44444.
5. If Ubuntu firewall is enabled, allow UDP 44444 **only on the private Wi-Fi interface** and from ESP32's IP.

## ROS node in its own Docker container (alongside existing thouzer_ros)

From the extracted project's root directory on Ubuntu:

```bash
sudo docker run --rm -it \
  --name thouzer_imu \
  --network host \
  -v "$PWD/ros_node:/opt/imu:ro" \
  thouzer-melodic:mqtt \
  bash -lc 'source /root/catkin_ws/devel/setup.bash && python /opt/imu/imu_udp_ros.py'
```

`thouzer_ros` container must already be running and providing the ROS master on 127.0.0.1:11311. For the first test keep MQTT launch `publish_odom:=false`, so the existing unverified odometry publisher stays off.

In a separate terminal:

```bash
sudo docker exec thouzer_ros rostopic hz /imu/data_raw
sudo docker exec thouzer_ros rostopic echo -n 1 /imu/data_raw
```

Expected frequency: around 50 Hz. At rest and with an upright level board, `angular_velocity.z` should be near 0 rad/s; acceleration should show roughly 9.81 m/s^2 along the vertical axis. Rotate the stationary robot **counterclockwise** (as viewed from above) and verify positive Z angular velocity (ROS convention). Do not command autonomous movement to test this.

## Important limitations

- Stock `odom.py` only reads IMU Z angular velocity, and integrates **commanded** linear speed from `/thouzer/formatted_vel`; these are not measured displacements. Wheel slip and commanded speed mismatch create drift.
- Stock `odom.py` rounds gyro rate in 0.05 rad/s steps, which can suppress slow turns; it also passes a degree/s value to ROS odometry `twist.angular.z` without conversion. Audit and fix before autonomous use.
- A gyro cannot supply reliable X/Y translation, and this example does **not** include magnetometer fusion or absolute heading. It publishes orientation as unavailable.
- Mapping and obstacle-aware navigation still require a usable `/scan` feed, without stealing the existing exclusive LiDAR connection.
- Wi-Fi delays and packet loss mean this is a prototype/diagnostic feed, not certified localization or safety-critical feedback. Require E-stop, controller-side command timeout, and a safe motion test procedure.
