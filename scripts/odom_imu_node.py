#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Melodic Python node to read IMU and encoder data from UART,
parse custom DATA_... format, and publish to /imu/data_raw,
/imu/mag, and /odom_raw topics at configurable rate.
Features:
 - Calibrate IMU: gyro (deg/s -> rad/s), mag (µT -> T), accel unchanged (m/s²)
 - Compute odometry from encoder counts via forward kinematics
"""
import rospy
import serial
import numpy as np
from math import pi
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf
import tf2_ros

# Global constants (SI units)
WHEEL_RADIUS = 0.041    # wheel radius in meters (0.041 m)
L = 0.180               # distance from wheel center to robot center in meters
SIG_DEG = [60.0, 180.0, 300.0]  # wheel angles in degrees
SIG = [angle * pi/180.0 for angle in SIG_DEG]  # convert to radians

def build_jacobian():
    # Forward kinematics matrix J (3x3)
    sin1, sin2, sin3 = np.sin(SIG)
    cos1, cos2, cos3 = np.cos(SIG)
    J = (WHEEL_RADIUS / 3.0) * np.array([
        # row 0: vx (ti?n lui)
        [-2*sin1,    -2*sin2,    -2*sin3],
        # row 1: vy (ngang tráiph?i)  d?o d?u so v?i tru?c
        [-2*cos1,    -2*cos2,    -2*cos3],
        # row 2: ? (xoay)  d?o d?u so v?i tru?c
        [-1.0/L,     -1.0/L,     -1.0/L]
    ])
    return J


class ImuOdomNode:
    def __init__(self):
        rospy.init_node('odom_imu_uart_node')
        # Params
        self.port = rospy.get_param('~port', '/dev/ttyTHS1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.timeout = rospy.get_param('~timeout', 1)
        self.rate_hz = rospy.get_param('~rate', 30)
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', 998)

        # Publishers
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom_raw', Odometry, queue_size=10)

        # Serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            rospy.loginfo("Opened serial port: %s and baundrate: %s", self.port, self.baudrate)
            self.ser.write(b'r')
            line = self.ser.readline().decode('utf-8', errors='ignore')
            rospy.loginfo(line)
        except serial.SerialException as e:
            rospy.logerr("Failed to open %s, error: %s",self.port, e)
            rospy.signal_shutdown("Serial port error")
            return

        # Precompute matrix
        self.J = build_jacobian()

        # State for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Ngay sau khi m? serial và tru?c khi tính toán gì
        now = rospy.Time.now().to_sec()
        self.prev_time = now
        self.prev_ticks = np.zeros(3, dtype=float)

        # --- thêm vào dây ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def parse_line(self, line):

        # Expect: DATA_gx_gy_gz_ax_ay_az_mx_my_mz_c1_c2_c3
        if not line.startswith('DATA_'):
            return None
        parts = line.strip().split('_')[1:]
        if len(parts) != 12:
            return None
        vals = []
        try:
            # first 9 floats, last 3 ints
            vals = [float(p) for p in parts[:9]]
            counts = [int(p) for p in parts[9:]]
            return vals + counts
        except ValueError:
            return None

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        DEG2RAD = pi/180.0

        while not rospy.is_shutdown():
            self.ser.write(b'o')
            line = self.ser.readline().decode('utf-8', errors='ignore')
            rospy.loginfo(line)
            data = self.parse_line(line)
            if data is None:
                rate.sleep()
                continue

            if rospy.is_shutdown():
                break

            # unpack
            gx, gy, gz, ax, ay, az, mx, my, mz, chead1, cback, chead2 = data
            now = rospy.Time.now()
            curr_time = now.to_sec()

            # --- Publish IMU ---
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            # gyro: deg/s -> rad/s
            imu.angular_velocity.x = gx * DEG2RAD
            imu.angular_velocity.y = gy * DEG2RAD
            imu.angular_velocity.z = gz * DEG2RAD
            # accel: m/s^2 unchanged
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            self.imu_pub.publish(imu)

            # --- Publish Magnetometer ---
            mag = MagneticField()
            mag.header.stamp = now
            mag.header.frame_id = 'imu_link'
            # µT -> T
            mag.magnetic_field.x = mx * 1e-6
            mag.magnetic_field.y = my * 1e-6
            mag.magnetic_field.z = mz * 1e-6
            self.mag_pub.publish(mag)

            # --- Compute Odometry ---
            dt = curr_time - self.prev_time
            if dt <= 0:
                dt = 1.0 / self.rate_hz
            self.prev_time = curr_time

            ticks_now = np.array([chead1, cback, chead2], dtype=float)
            delta_ticks = ticks_now - self.prev_ticks
            self.prev_ticks = ticks_now

            # omega_i (rad/s)
            omega = (2.0 * pi * delta_ticks) / (self.ticks_per_rev * dt)

            vel = self.J.dot(omega)
            vx, vy, omega_z = vel.tolist()

            # Integrate pose
            dx = (vx * np.cos(self.theta) - vy * np.sin(self.theta)) * dt
            dy = (vx * np.sin(self.theta) + vy * np.cos(self.theta)) * dt
            dtheta = omega_z * dt
            self.x += dx
            self.y += dy
            self.theta += dtheta
            # publish odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            # pose
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            odom.pose.pose.orientation = Quaternion(*quat)
            # twist
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = omega_z
            self.odom_pub.publish(odom)

            rate.sleep()

if __name__ == '__main__':
    try:
        node = ImuOdomNode()
        if not rospy.is_shutdown():
            node.spin()
    except rospy.ROSInterruptException:
        pass

