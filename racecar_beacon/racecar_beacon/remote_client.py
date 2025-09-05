#!/usr/bin/env python3

import socket
from struct import unpack

HOST = str("10.0.1.1")
PORT = int(65432)
MAX_BYTES = 1024

def main():
    while True:
        cmd = input("Enter command (RPOS/OBSF/RBID): ").strip().upper()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            s.connect((HOST, PORT))
            s.sendall(cmd.encode('ascii'))  # send 4 ASCII characters
            while True:
                data, addr = s.recvfrom(MAX_BYTES)
                if data is not None:
                    if cmd == "RPOS":
                        msg = unpack('fff',data)
                    elif cmd == "OBSF":
                        msg = unpack('I',data)[0]
                    elif cmd == "RBID":
                        msg = unpack('I',data)[0]
                    print(msg)
                    break
            
        except KeyboardInterrupt:
            pass
        finally:
            s.close()


if __name__ == "__main__":
    main()
