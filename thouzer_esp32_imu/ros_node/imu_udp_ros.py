#!/usr/bin/env python
# ROS 1 Melodic (Python 2 compatible) ESP32 MPU9250 UDP -> sensor_msgs/Imu.
# Expected packet: sequence,ax,ay,az,gx,gy,gz
# Accelerations are m/s^2 and gyro rates are rad/s.
# Does not produce orientation, odometry or navigation safety logic.

from __future__ import print_function
import socket
import rospy
from sensor_msgs.msg import Imu


def main():
    rospy.init_node('esp32_imu_udp')
    port = int(rospy.get_param('~port', 44444))
    frame = rospy.get_param('~frame_id', 'imu_link')
    allowed_ip = rospy.get_param('~allowed_ip', '')  # Optional whitelist
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=20)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(0.2)
    rospy.loginfo('Listening for ESP32 MPU9250 UDP packets on port %d', port)
    last_seq = None
    while not rospy.is_shutdown():
        try:
            packet, source = sock.recvfrom(256)
        except socket.timeout:
            continue
        except socket.error as e:
            rospy.logwarn('UDP socket error: %s', str(e))
            continue
        if allowed_ip and source[0] != allowed_ip:
            continue
        try:
            parts = packet.decode('ascii').strip().split(',')
            if len(parts) != 7:
                raise ValueError('expected 7 fields')
            seq = int(parts[0])
            ax, ay, az, gx, gy, gz = [float(x) for x in parts[1:]]
        except (ValueError, UnicodeError) as e:
            rospy.logwarn_throttle(5, 'Malformed ESP32 packet: %s', str(e))
            continue

        if last_seq is not None and seq > last_seq + 1:
            rospy.logwarn_throttle(10, 'ESP32 UDP packets were lost')
        last_seq = seq
        msg = Imu()
        msg.header.stamp = rospy.Time.now() # Approx. receive time, not sensor time
        msg.header.frame_id = frame
        msg.orientation_covariance[0] = -1.0 # No orientation estimate
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        # The remaining covariances are zero (unknown). Calibrate for production.
        pub.publish(msg)
    sock.close()


if __name__ == '__main__':
    main()
