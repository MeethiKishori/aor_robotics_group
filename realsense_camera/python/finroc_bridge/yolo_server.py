"""
TCP YOLO server for Finroc.

Finroc (mPercept) sends a JPEG-encoded camera frame; this server runs the
Ultralytics models (yolov8n = person, best.pt = Unitree robot) and sends the
bounding boxes back. Finroc feeds them into its risk pipeline.

Wire protocol (little-endian, localhost, same arch):
  request  (Finroc -> here):  [uint32 jpeg_len][jpeg bytes]
  response (here -> Finroc):  [uint32 count] then count x
                              [int32 x][int32 y][int32 w][int32 h]
                              [float32 conf][int32 label_len][label bytes]

Run (after recreating the venv):
    cd ~/Finroc/finroc_projects_scout/realsense_camera
    source realsense_env/bin/activate
    python3 python/finroc_bridge/yolo_server.py
"""

import os
import socket
import struct
import sys

import numpy as np
import cv2
import torch
from ultralytics import YOLO

HOST = "127.0.0.1"
PORT = 5555
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "..", "models"))

HUMAN_MODEL = os.path.join(MODELS_DIR, "yolov8n.pt")   # COCO, class 0 = person
ROBOT_MODEL = os.path.join(MODELS_DIR, "best.pt")      # custom Unitree robot
HUMAN_CONF = 0.20
ROBOT_CONF = 0.30


def recvall(conn, n):
    """Read exactly n bytes or return None if the connection closed."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def detect(model_human, model_robot, frame):
    """Return list of (x, y, w, h, conf, label)."""
    dets = []
    for b in model_human(frame, classes=[0], conf=HUMAN_CONF, device=DEVICE, verbose=False)[0].boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        dets.append((x1, y1, x2 - x1, y2 - y1, float(b.conf[0]), "person"))
    for b in model_robot(frame, conf=ROBOT_CONF, device=DEVICE, verbose=False)[0].boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        dets.append((x1, y1, x2 - x1, y2 - y1, float(b.conf[0]), "robot"))
    return dets


def pack_response(dets):
    resp = struct.pack("<I", len(dets))
    for (x, y, w, h, conf, label) in dets:
        lb = label.encode("ascii", "ignore")
        resp += struct.pack("<iiiifi", x, y, w, h, conf, len(lb)) + lb
    return resp


def main():
    print(f"Device: {DEVICE}")
    print(f"Loading models:\n  {HUMAN_MODEL}\n  {ROBOT_MODEL}")
    model_human = YOLO(HUMAN_MODEL)
    model_robot = YOLO(ROBOT_MODEL)
    model_human.to(DEVICE)
    model_robot.to(DEVICE)
    # Warm up so the first real frame isn't slow (GPU kernels / graph compile).
    warm = np.zeros((640, 640, 3), dtype=np.uint8)
    detect(model_human, model_robot, warm)
    print("models warmed up, ready")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"YOLO bridge listening on {HOST}:{PORT}  (Ctrl+C to stop)")

    while True:
        conn, addr = srv.accept()
        print("Finroc connected:", addr)
        try:
            while True:
                hdr = recvall(conn, 4)
                if hdr is None:
                    break
                (jpeg_len,) = struct.unpack("<I", hdr)
                data = recvall(conn, jpeg_len)
                if data is None:
                    break
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                dets = detect(model_human, model_robot, frame) if frame is not None else []
                conn.sendall(pack_response(dets))
        except (ConnectionError, OSError) as e:
            print("connection error:", e)
        finally:
            conn.close()
            print("Finroc disconnected; waiting for reconnect")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
        sys.exit(0)
