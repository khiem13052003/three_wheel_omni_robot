#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import sys
import select
from threading import Lock, Thread

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter')

        self.actual_path = []
        self.reference_path = []
        self.goal_received = False
        self.lock = Lock()

        # Subscribers
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.plan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Start a thread to listen for keypresses
        t = Thread(target=self.key_listener)
	t.daemon = True
	t.start()


        rospy.loginfo("Trajectory plotter is running. Press 'e' to plot when ready.")
        rospy.spin()

    def amcl_callback(self, msg):
        if not self.goal_received:
            return
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.actual_path.append((x, y))

    def plan_callback(self, msg):
        if not self.goal_received:
            return
        with self.lock:
            self.reference_path = [(pose.pose.position.x, pose.pose.position.y)
                                   for pose in msg.poses]

    def goal_callback(self, msg):
        rospy.loginfo("Received new goal. Start collecting data.")
        with self.lock:
            self.goal_received = True
            self.actual_path = []
            self.reference_path = []

    def key_listener(self):
        # Configure stdin to non-blocking
        rospy.loginfo("Press 'e' and Enter to plot trajectories.")
        while not rospy.is_shutdown():
            # Use select to wait for input
            if select.select([sys.stdin], [], [], 0.1)[0]:
                char = sys.stdin.readline().strip()
                if char.lower() == 'e':
                    rospy.loginfo("Key 'e' pressed. Plotting trajectories...")
                    with self.lock:
                        self.plot_trajectories()
            rospy.sleep(0.1)

    def plot_trajectories(self):
        if not self.reference_path or not self.actual_path:
            rospy.logwarn("Không d? d? li?u d? v?.")
            return

        ref_x, ref_y = zip(*self.reference_path)
        act_x, act_y = zip(*self.actual_path)

        plt.figure()
        plt.plot(ref_x, ref_y, 'b-', label="Planned Path (/NavfnROS/plan)")
        plt.plot(act_x, act_y, 'r--', label="Actual Path (AMCL)")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Robot Trajectory")
        plt.legend()
        plt.grid()
        plt.axis("equal")
        plt.show()

if __name__ == "__main__":
    try:
        TrajectoryPlotter()
    except rospy.ROSInterruptException:
        pass

