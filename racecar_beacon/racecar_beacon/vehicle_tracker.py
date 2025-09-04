#!/usr/bin/env python3

import socket
from struct import unpack, calcsize

"""
NOTES:

- This process MUST listen to a different port than the RemoteRequest client;
- A socket MUST be closed BEFORE exiting the process.
"""

PORT = int(65432)
BIND_ADDR = ""
MAX_BYTES = 2048

# Struct format:
STRUCT_FMT  = "!4sHBBfffI"
STRUCT_SIZE = calcsize(STRUCT_FMT)


def main():
    
    # UDP client socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((BIND_ADDR, PORT))

        while True:
            data, addr = sock.recvfrom(MAX_BYTES)
            src_ip, src_port = addr

            if len(data) == STRUCT_SIZE:
                continue

            unpacked_data = unpack(STRUCT_FMT, data)
            print(f"Received {unpacked_data} from {src_ip}:{src_port}")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == "__main__":
    main()
