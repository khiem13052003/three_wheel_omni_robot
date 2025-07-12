#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket

# C?ng mà node này s? l?ng nghe broadcast
UDP_PORT = 5005
# N?i dung tr? l?i khi du?c h?i
MESSAGE = "robot-jetson"

# T?o socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))  # L?ng nghe t?t c? d?a ch?

print("?? Listening for broadcast on UDP port {}".format(UDP_PORT))

while True:
    try:
        data, addr = sock.recvfrom(1024)
        decoded_data = data.decode('utf-8')
        print("?? Received from {}: {}".format(addr, decoded_data))

        # N?u nh?n du?c yêu c?u t? app
        if decoded_data == "who-is-there":
            hostname = socket.gethostname()
            response = "{}|{}".format(MESSAGE, hostname)
            sock.sendto(response.encode('utf-8'), addr)
            print("?? Replied to {} with: {}".format(addr, response))

    except Exception as e:
        print("? Error: {}".format(e))
