#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import struct
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def setup_serial(port="/dev/ttyUSB0", baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.2)
        rospy.loginfo("Serial port {} opened successfully".format(port))
        return ser
    except serial.SerialException as e:
        rospy.logerr("Could not open serial port {}: {}".format(port, e))
        return None

def send_command(ser, command):
    ser.write(command)
    rospy.loginfo("Sent command: {}".format(command))

def read_packet(ser):
    while not rospy.is_shutdown():
        packet = ser.read(22)
        if len(packet) == 22:
            if ord(packet[0]) == 0xFA:
                return packet
            else:
                rospy.logwarn("Invalid start byte: {}, resyncing...".format(hex(ord(packet[0]))))
        else:
            rospy.logwarn("Received packet with invalid length: {} bytes, resyncing...".format(len(packet)))
        byte = ser.read(1)
        if not byte:
            rospy.logerr("No data received from LiDAR, check connection")
            return None

def parse_packet(packet):
    if packet is None:
        rospy.logwarn("Packet is None, likely due to read failure")
        return None, None
    angle_raw = ord(packet[1])
    if 0xA0 <= angle_raw <= 0xF9:
        angle_base = (angle_raw - 0xA0) * 4
        distances = []
        for i in range(4):
            offset = 4 + i * 4
            distance = struct.unpack('<H', packet[offset:offset+2])[0]
            if distance < 15000:
                distances.append(distance / 1000.0)
            else:
                distances.append(float('nan'))
        return angle_base, distances
    else:
        rospy.logwarn("Invalid angle byte: {}, expected 0xA0 to 0xF9".format(hex(angle_raw)))
    return None, None

def publish_lidar_data():
    rospy.init_node('lds006_lidar_node', anonymous=True)
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    ser = setup_serial()
    if ser is None:
        return
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    send_command(ser, "startlds$")
    rospy.sleep(2)  # Chờ 2 giây để LiDAR khởi động

    rate = rospy.Rate(10)  # 10 Hz
    scan_msg = LaserScan()
    scan_msg.header.frame_id = "laser"
    scan_msg.range_min = 0.12
    scan_msg.range_max = 15.0
    scan_msg.angle_min = np.deg2rad(0)
    scan_msg.angle_max = np.deg2rad(359)

    try:
        while not rospy.is_shutdown():
            scan_data = [float('nan')] * 360
            valid_packets = 0
            for packet_idx in range(90):
                packet = read_packet(ser)
                angle, dist = parse_packet(packet)
                if angle is not None and dist is not None:
                    for i in range(4):
                        index = (angle + i) % 360
                        scan_data[index] = dist[i]
                    valid_packets += 1
                else:
                    rospy.logwarn("Failed to parse packet {}/90".format(packet_idx + 1))

            valid_data_count = sum(1 for d in scan_data if not np.isnan(d))
            if valid_data_count > 0:
                rospy.loginfo("Scan contains {}/360 valid data points".format(valid_data_count))
            else:
                rospy.logwarn("Scan contains no valid data!")

            scan_msg.ranges = scan_data
            scan_msg.angle_increment = np.deg2rad(360.0 / len(scan_msg.ranges))
            scan_msg.scan_time = 1.0 / 10.0
            scan_msg.time_increment = scan_msg.scan_time / len(scan_msg.ranges)
            scan_msg.header.stamp = rospy.Time.now()

            pub.publish(scan_msg)
            rate.sleep()

    except Exception as e:
        rospy.logerr("Error in LiDAR processing: {}".format(e))
    finally:
        if ser and ser.is_open:
            send_command(ser, "stoplds$")
            ser.flush()         
            # làm sạch buffer
            ser.flushInput()
            ser.flushOutput()
            # rồi mới close
            ser.close()
            rospy.loginfo("LiDAR stopped and serial port closed")

if __name__ == "__main__":
    try:
        publish_lidar_data()
    except rospy.ROSInterruptException:
        pass
