#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import numpy as np

from three_wheel_omni_robot.srv import ForwardKinematics, ForwardKinematicsResponse
from three_wheel_omni_robot.srv import InverseKinematics, InverseKinematicsResponse



class RobotKinematics(object):
    def __init__(self):
        # Thông số hình học robot; bạn có thể đưa qua param server nếu muốn
        # Bán kính bánh (mm) và khoảng cách L (mm) giống code gốc
        # Góc đặt bánh (độ)
        sig1 = 60
        sig2 = 180
        sig3 = 300
        wheel_radius = 41
        L = 180

        # Chuyển sang radian
        self.sig1 = np.deg2rad(sig1)
        self.sig2 = np.deg2rad(sig2)
        self.sig3 = np.deg2rad(sig3)
        self.wheel_radius = wheel_radius
        self.l = L

        # Tạo ma trận cho forward/inverse nếu bạn muốn precompute
        # Nhưng ở đây tính trực tiếp trong hàm.
        rospy.loginfo("RobotKinematics initialized: sig (deg)=({}, {}, {}), R={} mm, L={} mm".format(
            sig1, sig2, sig3, wheel_radius, L))

    def forward_kinematics(self, omegas):
        """
        Nhận omegas: list hoặc array length 3 [omega1, omega2, omega3] (rad/s)
        theta_deg: góc hiện tại (độ)
        Trả về np.array [v_x, v_y, omega_z] (mm/s, mm/s, rad/s)
        """
        if len(omegas) != 3:
            raise ValueError("ForwardKinematics: omegas phải có độ dài 3")
        # Ma trận J body velocities: theo code gốc nhưng cần đúng hệ:
        # Ở code bạn: 

        J = (self.wheel_radius / 3.0) * np.array([
            [-2.0 * np.sin(self.sig1), -2.0 * np.sin(self.sig2), -2.0 * np.sin(self.sig3)],
            [ 2.0 * np.cos(self.sig1),  2.0 * np.cos(self.sig2),  2.0 * np.cos(self.sig3)],
            [ 1.0 / self.l,             1.0 / self.l,             1.0 / self.l]
        ])
        omegas_arr = np.array(omegas, dtype=float)
        v_body = J.dot(omegas_arr)  # [v_x_body, v_y_body, omega_z]
        return v_body  # numpy array length 3

    def inverse_kinematics(self, v_body):
        """
        Nhận v_body: list hoặc array length 3 [v_x, v_y, omega_z] (mm/s, mm/s, rad/s)
        theta_deg: góc hiện tại (độ)
        Trả về list length 3: [omega1, omega2, omega3] (rad/s)
        """
        if len(v_body) != 3:
            raise ValueError("InverseKinematics: v_body phải có độ dài 3")

        # Ma trận J_inv: 
        J_inv = (1.0 / self.wheel_radius) * np.array([
            [-np.sin(self.sig1),  np.cos(self.sig1), self.l],
            [-np.sin(self.sig2),  np.cos(self.sig2), self.l],
            [-np.sin(self.sig3),  np.cos(self.sig3), self.l]
        ])
        omegas = J_inv.dot(v_body)
        # Trả về list float
        return [float(omegas[0]), float(omegas[1]), float(omegas[2])]

def handle_forward(req):
    """
    Callback cho service ForwardKinematics.
    req.omegas: float64[]
    """
    try:
        rospy.loginfo("handle forward_kinematics: %s", req.omegas)
        result = robot_kin.forward_kinematics(req.omegas)
        # Chuyển numpy array sang list
        return ForwardKinematicsResponse(list(result))
    except Exception as e:
        rospy.logerr("Error in forward kinematics: %s", str(e))
        # Trả mảng zeros khi lỗi
        return ForwardKinematicsResponse([0.0, 0.0, 0.0])

def handle_inverse(req):
    """
    Callback cho service InverseKinematics.
    req.v_body: float64[] length 3
    """
    try:
        rospy.loginfo("handle inverse_kinematics: %s", req.v_body)
        result = robot_kin.inverse_kinematics(req.v_body)
        return InverseKinematicsResponse(result)
    except Exception as e:
        rospy.logerr("Error in inverse kinematics: %s", str(e))
        return InverseKinematicsResponse([0.0, 0.0, 0.0])

if __name__ == '__main__':
    rospy.init_node('kinematics_service_node')
    # Khởi RobotKinematics (đọc param nếu cần)
    robot_kin = RobotKinematics()

    # Tạo service
    service_fwd = rospy.Service('forward_kinematics', ForwardKinematics, handle_forward)
    service_inv = rospy.Service('inverse_kinematics', InverseKinematics, handle_inverse)
    rospy.loginfo("Kinematics services ready: forward_kinematics, inverse_kinematics")
    rospy.spin()
