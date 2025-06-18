#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf
from geometry_msgs.msg import Pose2D, Quaternion, Point, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

class Pose2DImuToOdom:
    def __init__(self):
        rospy.init_node('pose2d_imu_to_odom')

        # Params
        self.pose2d_topic = rospy.get_param('~/pose2d_topic', '/pose2D')
        self.imu_topic    = rospy.get_param('~/imu_topic', '/imu/data')
        self.odom_topic   = rospy.get_param('~/odom_topic', '/odom')
        self.frame_id     = rospy.get_param('~/odom_frame_id', 'odom')
        self.child_frame  = rospy.get_param('~/base_frame_id', 'base_link')

        # Subscribers
        self.pose2d_sub = rospy.Subscriber(self.pose2d_topic, Pose2D, self.cb_pose2d)
        self.imu_sub    = rospy.Subscriber(self.imu_topic, Imu, self.cb_imu)

        # Publisher
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # State lưu pose trước để tính dx/dt
        self.prev_pose = None  # Pose2D
        self.prev_time = None  # rospy.Time
        # Lưu angular velocity từ IMU mới nhất
        self.latest_imu = None

    def cb_imu(self, msg):
        # Lưu angular vel.z để dùng khi publish odom
        self.latest_imu = msg

    def cb_pose2d(self, msg):
        now = rospy.Time.now()
        # Nếu chưa có previous, chỉ lưu và return
        if self.prev_pose is None or self.prev_time is None:
            self.prev_pose = msg
            self.prev_time = now
            return

        dt = (now - self.prev_time).to_sec()
        if dt <= 0:
            return

        # Tính dx, dy
        dx = msg.x - self.prev_pose.x
        dy = msg.y - self.prev_pose.y
        # Tính tốc độ world
        vx_world = dx / dt
        vy_world = dy / dt
        # Tính body-frame velocity theo θ hiện tại (msg.theta)
        theta = msg.theta
        vx_body =  math.cos(theta)*vx_world + math.sin(theta)*vy_world
        vy_body = -math.sin(theta)*vx_world + math.cos(theta)*vy_world

        # Lấy angular velocity z từ IMU nếu có, ngược lại tính từ Δθ/Δt
        if self.latest_imu is not None:
            wz = self.latest_imu.angular_velocity.z
        else:
            wz = (msg.theta - self.prev_pose.theta) / dt

        # Tạo Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame

        # Pose: từ Pose2D → quaternion
        q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        odom.pose.pose.position = Point(msg.x, msg.y, 0.0)
        odom.pose.pose.orientation = Quaternion(*q)
        # Covariance: nếu có từ Pose2D (ví dụ gói phát pose gửi covariance), map vào odom.pose.covariance.
        # Nếu không, để mặc định là unkown (0) hoặc bạn tự đặt:
        # Example: odom.pose.covariance = [0.1,0,...] 6x6 flattened
        # Tương tự odom.twist.covariance.

        # Twist
        odom.twist.twist = Twist(
            Vector3(vx_body, vy_body, 0.0),
            Vector3(0.0, 0.0, wz)
        )

        # Publish
        self.odom_pub.publish(odom)


        # Cập nhật prev
        self.prev_pose = msg
        self.prev_time = now

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = Pose2DImuToOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
