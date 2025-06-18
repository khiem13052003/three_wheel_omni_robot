#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from geometry_msgs.msg import Twist
from three_wheel_omni_robot.srv import InverseKinematics, InverseKinematicsRequest
import numpy as np



class CmdVelToArduino:
    def __init__(self):
        rospy.init_node('cmd_vel_to_arduino')

        # Tham số Serial
        serial_port = rospy.get_param('~/port_arduino', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~/baudrate', 115200)
        self.cmd_prefix = ":GO_"
        self.cmd_suffix = ":"

        self.sig1 = np.deg2rad(rospy.get_param('~/sig1', 60))
        self.sig2 = np.deg2rad(rospy.get_param('~/sig2', 180))
        self.sig3 = np.deg2rad(rospy.get_param('~/sig3', 300))
        self.wheel_radius = rospy.get_param('~/wheel_radius', 0.041)
        self.l = rospy.get_param('~/L', 0.18)

        # Mở Serial
        try:
            self.ser = serial.Serial(serial_port, baudrate=baudrate, timeout=1.0)
            rospy.loginfo("Opened serial port %s @ %d", serial_port, baudrate)
        except serial.SerialException as e:
            rospy.logerr("Cannot open serial port %s: %s", serial_port, e)
            rospy.signal_shutdown("Serial port error")
            return

        # # Service proxy InverseKinematics
        # service_name = 'inverse_kinematics'
        # rospy.loginfo("Waiting for service %s ...", service_name)
        # try:
        #     rospy.wait_for_service(service_name, timeout=10.0)
        # except rospy.ROSException:
        #     rospy.logerr("Service %s not available", service_name)
        #     rospy.signal_shutdown("Missing service")
        #     return
        # self.inv_kin = rospy.ServiceProxy(service_name, InverseKinematics)
        # rospy.loginfo("Connected to service %s", service_name)



        # Subscribe cmd_vel
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.cb_cmd_vel)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Ready, listening on /cmd_vel")

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

    def cb_cmd_vel(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # # Gọi service với v_body = [vx, vy, wz]
        # req = InverseKinematicsRequest()
        # req.v_body = [vx, vy, wz]
        # try:
        #     resp = self.inv_kin(req)
        # except rospy.ServiceException as e:
        #     rospy.logerr("InverseKinematics call failed: %s", e)
        #     return
        # # Lấy omegas
        # omegas = resp.omegas
        # if not omegas or len(omegas) != 3:
        #     rospy.logwarn("InverseKinematicsResponse invalid length: %s", str(omegas))
        #     return
        w1, w2, w3 = self.inverse_kinematics([vx,vy,wz])

        # Format chuỗi ":GO_<w1>_<w2>_<w3>:"
        try:
            s1 = "{:.3f}".format(w1)
            s2 = "{:.3f}".format(w2)
            s3 = "{:.3f}".format(w3)
        except Exception:
            s1, s2, s3 = str(w1), str(w2), str(w3)
        cmd = "{}{}_{}_{}{}".format(self.cmd_prefix, s1, s2, s3, self.cmd_suffix)
        line = cmd + "\n"
        try:
            self.ser.write(line.encode('utf-8'))
            rospy.loginfo("Sent to Arduino: %s", line.strip())
        except serial.SerialException as e:
            rospy.logerr("Serial write failed: %s", e)

    def on_shutdown(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.close()
                rospy.loginfo("Closed serial port")
            except:
                pass

if __name__ == '__main__':
    try:
        node = CmdVelToArduino()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
