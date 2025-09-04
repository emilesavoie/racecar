#!/usr/bin/env python3
import socket
from struct import unpack, calcsize
from datetime import datetime

PORT = 65431
MAX_BYTES = 1024

STRUCT_FMT  = "!4sHBBfffI"
STRUCT_SIZE = calcsize(STRUCT_FMT)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        sock.bind(("", PORT))

        print(f"Listening on 0.0.0.0:{PORT}")
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
