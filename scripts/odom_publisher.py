#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

def quaternion_to_yaw(q):
    # q: geometry_msgs/Quaternion
    # Tr? v? yaw (theta) trong [-pi, pi]
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return euler[2]

def normalize_angle(angle):
    # normalize v? [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

class PoseStampedImuToOdom:
    def __init__(self):
        rospy.init_node('pose_stamped_imu_to_odom')

        # Params
        self.pose_topic   = rospy.get_param('~pose_topic', '/pose_stamped')
        self.imu_topic    = rospy.get_param('~imu_topic', '/imu/data')
        self.odom_topic   = rospy.get_param('~odom_topic', '/odom_raw')
        self.frame_id     = rospy.get_param('~odom_frame_id', 'odom')
        self.child_frame  = rospy.get_param('~base_frame_id', 'base_link')

        # Subscribers
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose)
        self.imu_sub  = rospy.Subscriber(self.imu_topic, Imu, self.cb_imu)

        # Publisher
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # State luu pose tru?c
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time = None

        # State luu IMU m?i nh?t
        self.latest_imu = None

    def cb_imu(self, msg):
        # Luu IMU toàn b?, d? l?y angular_velocity.z
        self.latest_imu = msg

    def cb_pose(self, msg):
        # L?y timestamp t? header, n?u 0 thì dùng now()
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            now = rospy.Time.now()
        else:
            now = stamp

        # Trích xu?t v? trí và yaw
        x = msg.pose.position.x
        y = msg.pose.position.y
        quat = msg.pose.orientation
        theta = quaternion_to_yaw(quat)

        # N?u chua có previous, ch? luu và return
        if self.prev_time is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_theta = theta
            self.prev_time = now
            return

        dt = (now - self.prev_time).to_sec()
        if dt <= 0:
            # th?i gian không tang, b? qua
            return

        # Tính delta
        dx = x - self.prev_x
        dy = y - self.prev_y
        # world-frame velocities
        vx_world = dx / dt
        vy_world = dy / dt
        # body-frame: rotate vào khung base_link: 
        # [vx_body; vy_body] = R^{-1}(theta) * [vx_world; vy_world]
        # v?i R(theta) = rotation from body to world
        vx_body =  math.cos(theta)*vx_world + math.sin(theta)*vy_world
        vy_body = -math.sin(theta)*vx_world + math.cos(theta)*vy_world

        # Angular velocity: uu tiên IMU n?u có
        if self.latest_imu is not None:
            wz = self.latest_imu.angular_velocity.z
        else:
            # tính t? delta yaw, normalize tru?c
            dtheta = normalize_angle(theta - self.prev_theta)
            wz = dtheta / dt

        # T?o Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame

        # Pose
        odom.pose.pose.position = Point(x, y, 0.0)
        odom.pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        # N?u mu?n set covariance, có th? ? dây. Ví d?:
        # cov_pose = [0.1]*36  # ví d?, 6x6 flattened
        # odom.pose.covariance = cov_pose
        # odom.twist.covariance = cov_pose

        # Twist
        odom.twist.twist = Twist(
            Vector3(vx_body, vy_body, 0.0),
            Vector3(0.0, 0.0, wz)
        )

        # Publish Odometry
        self.odom_pub.publish(odom)

        # Broadcast TF: t? odom_frame -> base_frame
        self.tf_broadcaster.sendTransform(
            (x, y, 0.0),
            (quat.x, quat.y, quat.z, quat.w),
            now,
            self.child_frame,
            self.frame_id
        )

        # C?p nh?t prev
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        self.prev_time = now

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = PoseStampedImuToOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass

