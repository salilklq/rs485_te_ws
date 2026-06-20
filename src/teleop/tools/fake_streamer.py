"""Emit synthetic MANUS keypoint datagrams for testing WITHOUT a glove.

Builds a simple kinematic human-hand model (21 MANO keypoints, meters) whose
fingers curl and whose thumb opposes, animated over time, and streams it over
UDP exactly like the C++ ManusKeypointStreamer would. Lets you exercise the full
retarget -> driver -> panel pipeline offline.

    python fake_streamer.py --host 127.0.0.1 --port 9001 [--mode wave|grasp|static]
"""
import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dexhand_teleop import protocol  # noqa: E402

# Approximate human right-hand geometry (meters), palm in +Y, curl toward -Z.
_MCP = {
    "thumb": np.array([0.030, 0.025, 0.0]),
    "index": np.array([0.022, 0.090, 0.0]),
    "middle": np.array([0.000, 0.095, 0.0]),
    "ring": np.array([-0.020, 0.090, 0.0]),
    "pinky": np.array([-0.040, 0.080, 0.0]),
}
# phalanx lengths [proximal, intermediate, distal]
_LEN = {
    "thumb": [0.035, 0.032, 0.028],
    "index": [0.040, 0.025, 0.022],
    "middle": [0.045, 0.028, 0.024],
    "ring": [0.040, 0.026, 0.022],
    "pinky": [0.033, 0.020, 0.018],
}
_ORDER = ["thumb", "index", "middle", "ring", "pinky"]
_IDX = {"thumb": protocol.THUMB, "index": protocol.INDEX, "middle": protocol.MIDDLE,
        "ring": protocol.RING, "pinky": protocol.PINKY}


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def synth_hand(flex, thumb_opp, is_right=True):
    """flex: 5 values in [0,1] (curl per finger), thumb_opp in [0,1]. -> (21,3)."""
    kp = np.zeros((21, 3))
    for fi, finger in enumerate(_ORDER):
        base = _MCP[finger].copy()
        lens = _LEN[finger]
        if finger == "thumb":
            # opposition rotates the whole thumb about Z toward the fingers
            opp = _rot_z(thumb_opp * 1.0)
            base = opp @ base
            direction = opp @ np.array([0.2, 0.6, 0.0])
            direction = direction / np.linalg.norm(direction)
            curl_axis = opp @ np.array([1.0, 0.0, 0.0])
            seg_angle = flex[fi] * 0.9
        else:
            direction = np.array([0.0, 1.0, 0.0])
            curl_axis = np.array([1.0, 0.0, 0.0])
            seg_angle = flex[fi] * 1.4

        pos = base.copy()
        idx = _IDX[finger]
        kp[idx[0]] = pos
        d = direction.copy()
        for k in range(3):
            # progressive curl: each joint bends by seg_angle
            ang = seg_angle * (1.0 if k > 0 else 0.6)
            if finger == "thumb":
                Rk = _rot_about(curl_axis, ang)
            else:
                Rk = _rot_x(ang)
            d = Rk @ d
            d = d / np.linalg.norm(d)
            pos = pos + d * lens[k]
            kp[idx[k + 1]] = pos
    if not is_right:
        kp[:, 0] *= -1.0
    return kp


def _rot_about(axis, angle):
    axis = axis / (np.linalg.norm(axis) + 1e-9)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9001)
    ap.add_argument("--rate", type=float, default=60.0)
    ap.add_argument("--mode", choices=["wave", "grasp", "static"], default="wave")
    ap.add_argument("--sides", default="right,left")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sides = []
    if "right" in args.sides:
        sides.append(protocol.SIDE_RIGHT)
    if "left" in args.sides:
        sides.append(protocol.SIDE_LEFT)

    seq = 0
    t0 = time.monotonic()
    period = 1.0 / args.rate
    print(f"streaming synthetic hands to {args.host}:{args.port} mode={args.mode}")
    while True:
        t = time.monotonic() - t0
        if args.mode == "static":
            flex = np.array([0.3, 0.5, 0.5, 0.5, 0.5]); opp = 0.5
        elif args.mode == "grasp":
            phase = 0.5 - 0.5 * math.cos(t * 1.2)  # 0..1
            flex = np.array([phase] * 5); opp = phase
        else:  # wave: each finger curls in sequence
            flex = np.array([0.5 - 0.5 * math.cos(t * 1.5 + i) for i in range(5)])
            opp = 0.5 - 0.5 * math.cos(t * 0.8)
        for side in sides:
            kp = synth_hand(flex, opp, is_right=(side == protocol.SIDE_RIGHT))
            frame = protocol.KeypointFrame(side=side, valid=True, seq=seq,
                                           stamp_us=int(t * 1e6), points=kp)
            sock.sendto(protocol.pack(frame), (args.host, args.port))
        seq += 1
        time.sleep(period)


if __name__ == "__main__":
    main()
