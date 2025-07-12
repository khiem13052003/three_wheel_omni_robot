#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import json
import rospkg
from three_wheel_omni_robot.srv import waypoint, waypointResponse

def handle_request(req):
    try:
        rp = rospkg.RosPack()
        file_path = os.path.join(rp.get_path("three_wheel_omni_robot"), "config/waypoints.json")

        with open(file_path, "r") as f:
            data = json.load(f)

        # Chuy?n d?i t? dict {name: {pose}} thành list [{name: name, ...}]
        formatted = []
        for name, val in data.items():
            formatted.append({
                "name": name,
                "position": val["position"],
                "orientation": val["orientation"]
            })

        json_string = json.dumps(formatted)
        return waypointResponse(json=json_string)

    except Exception as e:
        rospy.logerr("Error reading JSON: %s", str(e))
        return waypointResponse(json="[]")

def main():
    rospy.init_node("waypoint_json_service_node")
    rospy.Service("/waypoint", waypoint, handle_request)
    rospy.loginfo("? Service /waypoint s?n sàng.")
    rospy.spin()

if __name__ == "__main__":
    main()

