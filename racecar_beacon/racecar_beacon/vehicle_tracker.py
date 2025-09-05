#!/usr/bin/env python3
import socket
from struct import unpack, calcsize
from datetime import datetime

PORT = 65431
MAX_BYTES = 1024
BIND_ADDR = ""

# This specifies the packet structure
STRUCT_FMT  = "fffI"
"""
Struct FMT
fff -> float32 (x, y, yaw)
I  -> uint32 (vehicle_id)
"""

STRUCT_SIZE = calcsize(STRUCT_FMT)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((BIND_ADDR, PORT))

        while True:
            data, addr = sock.recvfrom(MAX_BYTES)
            src_ip, src_port = addr
            ts = datetime.now().strftime("%H:%M:%S")

            if len(data) >= STRUCT_SIZE:
                x, y, _yaw, _veh_id = unpack(STRUCT_FMT, data[:STRUCT_SIZE])
                print(f"{ts} {src_ip}:{src_port}  x={x:.3f}  y={y:.3f} theta={_yaw:.3f}  veh_id={_veh_id}  {len(data)}B")
            else:
                print(f"{ts} {src_ip}:{src_port}  {len(data)}B (too short)")

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == "__main__":
    main()
