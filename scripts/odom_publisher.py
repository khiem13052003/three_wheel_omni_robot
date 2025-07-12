#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Melodic Python node that listens to /pose_stamped and republishes as /odom_raw at 35Hz.
Subscribes:
  /pose_stamped  (geometry_msgs/PoseStamped)
Publishes:
  /odom_raw      (nav_msgs/Odometry)
"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def pose_callback(msg, odom_pub):
    # Create Odometry message
    odom = Odometry()
    # Use the same timestamp as incoming PoseStamped
    odom.header.stamp = msg.header.stamp
    # Preserve frame_id from incoming message or set default
    odom.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
    # Child frame remains robot base
    odom.child_frame_id = 'base_link'

    # Copy pose data
    odom.pose.pose = msg.pose

    # TODO: compute or set twist and covariance if needed

    odom_pub.publish(odom)


def main():
    rospy.init_node('pose_to_odom', anonymous=False)

    # Publisher for odometry
    odom_pub = rospy.Publisher('/odom_raw1', Odometry, queue_size=10)

    # Subscriber to PoseStamped
    rospy.Subscriber('/pose_stamped', PoseStamped, pose_callback, callback_args=odom_pub)

    rospy.loginfo('pose_to_odom node started, converting /pose_stamped to /odom_raw at 35Hz')

    # Spin to keep callbacks alive
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

