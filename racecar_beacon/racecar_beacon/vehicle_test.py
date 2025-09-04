#!/usr/bin/env python3
import socket
from struct import pack
import time
import math

BROADCAST_IP = "255.255.255.255"   # or your LAN broadcast (ex: 192.168.1.255)
PORT = 65431

STRUCT_FMT = "!4sHBBfffI"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

seq = 0
while True:
    x = math.sin(time.time()) * 5
    y = math.cos(time.time()) * 5
    yaw = math.radians((time.time() * 30) % 360)

    packet = pack(STRUCT_FMT,
                  b"POS1",   # magic
                  1,         # vehicle_id
                  0,         # flags
                  0,         # reserved
                  x, y, yaw, # floats
                  seq)       # sequence counter

    sock.sendto(packet, (BROADCAST_IP, PORT))
    print(f"sent: {x:.2f}, {y:.2f}")
    seq += 1
    time.sleep(1)
