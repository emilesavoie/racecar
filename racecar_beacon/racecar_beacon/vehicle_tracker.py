#!/usr/bin/env python3
import socket
from struct import unpack, calcsize
from datetime import datetime

PORT = 65431
MAX_BYTES = 1024
BIND_ADDR = ""

# This specifies the packet structure for python
STRUCT_FMT  = "!4sHBBfffI"
"""
Struct FMT
4s -> 4 byte string (magic = "POS1")
H  -> uint16 (vehicle_id)
B  -> uint8 (flags)
B  -> uint8 (flags)
fff -> float32 (x, y, yaw)
I  -> uint32 (Sequence number)
"""
STRUCT_SIZE = calcsize(STRUCT_FMT)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((BIND_ADDR, PORT))

        while True:
            data, addr = sock.recvfrom(MAX_BYTES)
            src_ip, src_port = addr
            ts = datetime.now().strftime("%H:%M:%S")

            if len(data) >= STRUCT_SIZE:
                _magic, _veh_id, _flags, _res, x, y, _yaw, _seq = unpack(STRUCT_FMT, data[:STRUCT_SIZE])
                print(f"{ts} {src_ip}:{src_port}  x={x:.3f}  y={y:.3f}")
            else:
                print(f"{ts} {src_ip}:{src_port}  {len(data)}B (too short)")

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == "__main__":
    main()
