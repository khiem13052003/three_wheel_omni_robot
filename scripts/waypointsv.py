#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from three_wheel_omni_robot.srv import waypoint, waypointResponse  # thay tên gói th?t

# B?n d? waypoint theo tên
waypoints = {
    "HOME": {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    "A": {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
    "B": {'x': 0.0, 'y': 2.0, 'yaw': 0.0},
    "C": {'x': 3.0, 'y': 2.0, 'yaw': 0.0}
}

def handle_go_to_waypoint(req):
    name = req.name
    if name not in waypoints:
        return GoToWaypointResponse(False, "Không có di?m tên: " + name)

    wp = waypoints[name]
    quat = quaternion_from_euler(0, 0, wp['yaw'])

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = wp['x']
    pose.pose.position.y = wp['y']
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    pub.publish(pose)
    rospy.loginfo("Sent goal to waypoint: " + name)
    return GoToWaypointResponse(True, "Ðã g?i t?i di?m: " + name)

def main():
    global pub
    rospy.init_node('waypoint_service_node')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    s = rospy.Service('/waypoint', waypoint, handle_go_to_waypoint)
    rospy.loginfo("Ðang ch?y service: /waypoint")
    rospy.spin()

if __name__ == "__main__":
    main()

