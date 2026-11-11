# Upgrade IMU ESP32-C3 Wi-Fi UDP — ROS1 Melodic

Tujuan: ESP32-C3 di SSID `RMS-10X-AAZ03` mengirim UDP 28-byte @50 Hz ke Ubuntu `192.168.212.102:5005`. Robot MQTT/LiDAR tetap di `192.168.212.1`.

## Isi paket
- `esp32c3_imu_udp/esp32c3_imu_udp.ino`: firmware Arduino, mempertahankan I2C dan frame 28 byte.
- `esp32c3_imu_udp/wifi_secrets.h.example`: salin ke `wifi_secrets.h`, isi password Wi-Fi sendiri. Jangan upload file rahasia.
- `rt_usb_9axisimu_driver/`: driver ROS1 dengan mode UDP baru. Mode USB lama masih tersedia.

## Pemasangan aman
1. **Backup source host**: `cd ~/thouzer_ws/thouzer_ros && cp -a rt_usb_9axisimu_driver rt_usb_9axisimu_driver.usb.backup`
2. Salin **isi folder** `rt_usb_9axisimu_driver` dari paket ke `~/thouzer_ws/thouzer_ros/rt_usb_9axisimu_driver/` di Ubuntu host (jangan ubah path). Karena bind mount, Docker membaca file yang sama. Build dengan `sudo docker exec thouzer_ros bash -lc 'source /opt/ros/melodic/setup.bash; cd /root/catkin_ws; catkin build rt_usb_9axisimu_driver'`.
3. Flash `esp32c3_imu_udp.ino` dari Arduino IDE (board ESP32-C3), setelah menyalin `wifi_secrets.h.example` menjadi `wifi_secrets.h` dan mengisi password. Pastikan Wi-Fi 2.4 GHz dan ESP32 dapat menjangkau Ubuntu.

## Pengujian tanpa mengubah launch robot permanen
- **Jangan jalankan dua node IMU dengan nama/topic sama.** Hentikan node IMU USB terlebih dahulu. Jika node IMU diluncurkan otomatis oleh `thouzer_robot.launch`, jangan memulai node UDP kedua; ubah include launch IMU di bringup menjadi launch UDP sebelum restart.
- Jalankan satu node: `roslaunch rt_usb_9axisimu_driver rt_usb_9axisimu_driver_udp.launch udp_port:=5005`
- Cek `rostopic hz /imu/data_raw` (target ~50 Hz).
- Tidak perlu `ports:` di Compose karena `network_mode: host`.
- Mode USB tetap: `roslaunch rt_usb_9axisimu_driver rt_usb_9axisimu_driver.launch port:=/dev/ttyACM0`.

## Penting
- Belum diuji pada perangkat fisik; uji sebelum melepas kabel.
- Magnetometer di firmware asal adalah placeholder 0, sehingga `/imu/mag` tetap nol.
- Jika ESP32 tidak mengirim data, UDP receiver menunggu dan sesekali log timeout; tidak akan mempublikasikan data palsu.
- `wifi_secrets.h` jangan dimasukkan ke git atau dikirim ke chat.
