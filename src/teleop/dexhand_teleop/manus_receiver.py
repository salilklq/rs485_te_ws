"""Receive MANUS keypoint datagrams (from the C++ ManusKeypointStreamer) over UDP.

Keeps the latest KeypointFrame per side. Thread-safe.
"""
import socket
import threading
import time
from typing import Dict, Optional, Tuple

from . import protocol


class ManusReceiver:
    def __init__(self, host: str = "127.0.0.1", port: int = 9001):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: Dict[int, protocol.KeypointFrame] = {}
        self._recv_time: Dict[int, float] = {}
        self._count = 0

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.2)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="manus-recv")
        self._thread.start()

    def _loop(self):
        bufsize = protocol.packet_size() + 64
        while self._running:
            try:
                data, _ = self._sock.recvfrom(bufsize)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = protocol.unpack(data)
            except ValueError:
                continue
            with self._lock:
                self._latest[frame.side] = frame
                self._recv_time[frame.side] = time.monotonic()
                self._count += 1

    def get_latest(self, side: int) -> Tuple[Optional[protocol.KeypointFrame], float]:
        """Return (frame, age_seconds). age is +inf if never received."""
        with self._lock:
            frame = self._latest.get(side)
            t = self._recv_time.get(side)
        if frame is None or t is None:
            return None, float("inf")
        return frame, time.monotonic() - t

    @property
    def packet_count(self) -> int:
        with self._lock:
            return self._count

    def stop(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
