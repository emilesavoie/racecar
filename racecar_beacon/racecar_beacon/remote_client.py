#!/usr/bin/env python3

import socket
from struct import unpack

HOST = str("172.20.10.2")
PORT = int(65432)

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.sendall(b"Hello, server!").encode('utf-8')
    data = s.recv(1024)
    print(f"Received {data!r}")
    s.close()

if __name__ == "__main__":
    main()
