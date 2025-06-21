# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

# import rospy
# import serial
# from sensor_msgs.msg import Imu, MagneticField
# from std_msgs.msg import Header

# def parse_imu_line(line):
#     """
#     Parse dòng có format: IMU_<gx>_<gy>_<gz>_<ax>_<ay>_<az>_<mx>_<my>_<mz>
#     Tr\u1ea3 v\u1ec1 tuple (gx,gy,gz, ax,ay,az, mx,my,mz) ho\u1eb7c None n\u1ebfu parse th\u1ea5t b\u1ea1i.
#     """
#     line = line.strip()
#     if not line.startswith("IMU_"):
#         return None
#     parts = line.split('_')
#     # parts[0]="IMU", c\u1ea7n t\u1ed5ng c\u1ed9ng 1 + 9 ph\u1ea7n
#     if len(parts) != 10:
#         return None
#     try:
#         nums = [float(p) for p in parts[1:]]
#         return tuple(nums)  # 9 giá tr\u1ecb
#     except ValueError:
#         return None

# def imu_serial_node():
#     rospy.init_node('imu_serial_node')
#     # Tham s\u1ed1 serial
#     port = rospy.get_param('~/port_arduino', '/dev/ttyUSB1')
#     baud = rospy.get_param('~/baudrate', 115200)
#     frame_id = rospy.get_param('~/frame_id', 'imu_link')
#     # Topic names
#     imu_topic = rospy.get_param('~/imu_topic', 'imu/data_raw')
#     mag_topic = rospy.get_param('~/mag_topic', 'imu/mag')  # có th\u1ec3 là 'imu/mag'
#     # Covariance
#     ori_cov = rospy.get_param('~/orientation_covariance', [0.1] + [0.0]*8)
#     angvel_cov = rospy.get_param('~/angular_velocity_covariance', [0.01] + [0.0]*8)
#     linacc_cov = rospy.get_param('~/linear_acceleration_covariance', [0.1] + [0.0]*8)
#     mag_cov = rospy.get_param('~/magnetic_field_covariance', [0.01] + [0.0]*8)

#     # M\u1edf serial
#     try:
#         ser = serial.Serial(port, baudrate=baud, timeout=1.0)
#         rospy.loginfo("Opened serial port %s @ %d", port, baud)
#     except serial.SerialException as e:
#         rospy.logerr("Cannot open serial port %s: %s", port, e)
#         return

#     imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=50)
#     mag_pub = rospy.Publisher(mag_topic, MagneticField, queue_size=50)

#     rate_hz = rospy.get_param('~rate', 70)  # t\u1ea7n s\u1ed1 \u0111\u1ecdc/publish t\u1ed1i \u0111a
#     rate = rospy.Rate(rate_hz)

#     while not rospy.is_shutdown():
#         try:
#             line = ser.readline().decode('utf-8', errors='ignore')
#         except Exception as e:
#             rospy.logwarn("Serial read error: %s", e)
#             continue
#         data = parse_imu_line(line)
#         if data is None:
#             continue
#         gx, gy, gz, ax, ay, az, mx, my, mz = data
#         now = rospy.Time.now()

#         # T\u1ea1o Imu msg
#         imu_msg = Imu()
#         imu_msg.header = Header(stamp=now, frame_id=frame_id)
#         # orientation không có: quaternion m\u1eb7c \u0111\u1ecbnh và covariance[0] = -1 \u0111\u1ec3 báo không dùng orientation
#         imu_msg.orientation.x = 0.0
#         imu_msg.orientation.y = 0.0
#         imu_msg.orientation.z = 0.0
#         imu_msg.orientation.w = 1.0
#         imu_msg.orientation_covariance = ori_cov

#         imu_msg.angular_velocity.x = gx
#         imu_msg.angular_velocity.y = gy
#         imu_msg.angular_velocity.z = gz
#         imu_msg.angular_velocity_covariance = angvel_cov

#         imu_msg.linear_acceleration.x = ax
#         imu_msg.linear_acceleration.y = ay
#         imu_msg.linear_acceleration.z = az
#         imu_msg.linear_acceleration_covariance = linacc_cov

#         imu_pub.publish(imu_msg)

#         # T\u1ea1o MagneticField msg
#         mag_msg = MagneticField()
#         mag_msg.header = Header(stamp=now, frame_id=frame_id)
#         # N\u1ebfu c\u1ea7n chuy\u1ec3n \u0111\u01a1n v\u1ecb: ví d\u1ee5 n\u1ebfu sensor tr\u1ea3 gauss, chuy\u1ec3n sang Tesla: 1 G = 1e-4 T
#         # Gi\u1ea3 s\u1eed d\u1eef li\u1ec7u \u0111ã \u1edf Tesla, n\u1ebfu không, hi\u1ec7u ch\u1ec9nh \u1edf \u0111ây.
#         mag_msg.magnetic_field.x = mx
#         mag_msg.magnetic_field.y = my
#         mag_msg.magnetic_field.z = mz
#         mag_msg.magnetic_field_covariance = mag_cov

#         mag_pub.publish(mag_msg)

#         rate.sleep()

#     # Khi shutdown, \u0111óng serial
#     try:
#         ser.close()
#         rospy.loginfo("Closed serial port")
#     except:
#         pass

# if __name__ == "__main__":
#     try:
#         imu_serial_node()
#     except rospy.ROSInterruptException:
#         pass
