"""
Standalone test client for yolo_server.py.

Sends one image over the same TCP protocol Finroc/mPercept uses, prints the
boxes the server returns, and saves an annotated copy so you can eyeball it.

Run (server must already be running in another terminal):
    python3 python/finroc_bridge/test_client.py <image.jpg>
    # e.g. a robot image from the dataset:
    python3 python/finroc_bridge/test_client.py ../unitree-go1-640/valid/IMG_20220317_140218_jpg.rf.2JEgwTtag9pzcZaM9YT0.jpg
"""

import socket
import struct
import sys

import numpy as np
import cv2

HOST, PORT = "127.0.0.1", 5555


def recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("server closed connection")
        buf += chunk
    return buf


def main():
    if len(sys.argv) < 2:
        print("usage: python3 test_client.py <image>")
        sys.exit(1)

    path = sys.argv[1]
    img = cv2.imread(path)
    if img is None:
        print("cannot read image:", path)
        sys.exit(1)

    ok, jpeg = cv2.imencode(".jpg", img)
    data = jpeg.tobytes()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    # request: [uint32 len][jpeg]
    s.sendall(struct.pack("<I", len(data)) + data)

    # response: [uint32 count] then count x [4*int32][float][int32 len][label]
    (count,) = struct.unpack("<I", recvall(s, 4))
    print(f"server returned {count} detection(s):")
    for _ in range(count):
        x, y, w, h, conf, label_len = struct.unpack("<iiiifi", recvall(s, 24))
        label = recvall(s, label_len).decode("ascii", "ignore") if label_len else "?"
        print(f"  {label:8s} conf={conf:.2f}  box=({x},{y},{w},{h})")
        color = (0, 255, 0) if label == "person" else (255, 0, 0)
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        cv2.putText(img, f"{label} {conf:.2f}", (x, max(12, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    s.close()

    out = "test_client_result.jpg"
    cv2.imwrite(out, img)
    print(f"annotated image saved to: {out}")


if __name__ == "__main__":
    main()
