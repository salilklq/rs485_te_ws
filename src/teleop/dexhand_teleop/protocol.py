"""UDP keypoint protocol shared between the C++ ManusKeypointStreamer and Python.

One datagram per hand per frame. All little-endian.

    magic    char[4]  = 'MNKP'
    version  uint8    = 1
    side     uint8    0 = left, 1 = right
    valid    uint8    1 = tracking valid this frame
    npts     uint8    = 21
    seq      uint32   frame counter (per side)
    stamp_us uint64   source timestamp in microseconds (0 if unknown)
    points   float32[npts*3]   (x,y,z) per keypoint

Keypoints use the MediaPipe/MANO 21-point order (see MANO_* below) and are given
in the MANUS raw-skeleton frame (meters when MANUS VUH unitScale = 1.0). The
Python side re-canonicalises them to the MANO frame, so the absolute orientation
of the source frame does not matter.
"""
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = b"MNKP"
VERSION = 1
NUM_POINTS = 21

SIDE_LEFT = 0
SIDE_RIGHT = 1
SIDE_NAME = {SIDE_LEFT: "left", SIDE_RIGHT: "right"}

# MediaPipe / MANO 21-keypoint indices
WRIST = 0
THUMB = [1, 2, 3, 4]      # CMC, MCP, IP, TIP
INDEX = [5, 6, 7, 8]      # MCP, PIP, DIP, TIP
MIDDLE = [9, 10, 11, 12]
RING = [13, 14, 15, 16]
PINKY = [17, 18, 19, 20]
FINGERTIPS = [4, 8, 12, 16, 20]  # thumb..pinky tips

_HEADER = struct.Struct("<4sBBBBIQ")  # magic,ver,side,valid,npts,seq,stamp_us
_HEADER_SIZE = _HEADER.size
_PACKET_SIZE = _HEADER_SIZE + NUM_POINTS * 3 * 4


@dataclass
class KeypointFrame:
    side: int
    valid: bool
    seq: int
    stamp_us: int
    points: np.ndarray  # (21, 3) float32


def pack(frame: KeypointFrame) -> bytes:
    pts = np.ascontiguousarray(frame.points, dtype="<f4")
    if pts.shape != (NUM_POINTS, 3):
        raise ValueError(f"points must be (21,3), got {pts.shape}")
    header = _HEADER.pack(MAGIC, VERSION, frame.side, 1 if frame.valid else 0,
                          NUM_POINTS, frame.seq & 0xFFFFFFFF, frame.stamp_us & 0xFFFFFFFFFFFFFFFF)
    return header + pts.tobytes()


def unpack(data: bytes) -> KeypointFrame:
    if len(data) < _PACKET_SIZE:
        raise ValueError(f"packet too short: {len(data)} < {_PACKET_SIZE}")
    magic, ver, side, valid, npts, seq, stamp_us = _HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError("bad magic")
    if npts != NUM_POINTS:
        raise ValueError(f"unexpected npts {npts}")
    pts = np.frombuffer(data, dtype="<f4", count=NUM_POINTS * 3,
                        offset=_HEADER_SIZE).reshape(NUM_POINTS, 3).astype(np.float64)
    return KeypointFrame(side=side, valid=bool(valid), seq=seq, stamp_us=stamp_us, points=pts)


def packet_size() -> int:
    return _PACKET_SIZE
