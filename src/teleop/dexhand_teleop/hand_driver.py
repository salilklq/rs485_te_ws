"""Drive one dexterous hand over RS485 (Modbus-RTU) and read its feedback.

Register map (per the protocol doc, _protocol_decoded.txt):
    0..5    position cmd   0..1000 (0=open/straight & thumb not opposed, 1000=closed/opposed)
    6..11   speed   cmd    0..1000
    12..17  force   cmd    0..1000
    18..30  fingertip/palm force feedback (grams, valid 750..3000)   [read only]
    31..40  joint angle feedback (unit 0.1 deg)                      [read only]
    41..46  motor position feedback 0..1000                          [read only]

A retargeted joint angle (rad) maps to a position register via the per-joint
{qmin,qmax,out_lo,out_hi,invert} in drive.yml.

Set port="" for dry-run (everything computed, nothing written to a serial port).
"""
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

try:
    import serial  # pyserial
except Exception:  # pragma: no cover - allow import without pyserial for tests
    serial = None

CONTROL_REG_START = 0
SPEED_REG_START = 6
FORCE_REG_START = 12
FEEDBACK_REG_START = 18
FEEDBACK_REG_COUNT = 29  # 18..46 inclusive

FINGER_NAMES = ["thumb_rot", "thumb_flex", "index", "middle", "ring", "pinky"]


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_write_multiple(slave: int, start: int, values: List[int]) -> bytes:
    frame = bytearray()
    frame += bytes([slave & 0xFF, 0x10, (start >> 8) & 0xFF, start & 0xFF,
                    (len(values) >> 8) & 0xFF, len(values) & 0xFF, len(values) * 2])
    for v in values:
        v = int(v) & 0xFFFF
        frame += bytes([(v >> 8) & 0xFF, v & 0xFF])
    crc = modbus_crc16(bytes(frame))
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(frame)


def build_read_holding(slave: int, start: int, count: int) -> bytes:
    frame = bytes([slave & 0xFF, 0x03, (start >> 8) & 0xFF, start & 0xFF,
                   (count >> 8) & 0xFF, count & 0xFF])
    crc = modbus_crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class SerialBus:
    """One serial handle, possibly shared by both hands on a multi-drop bus."""

    _buses: Dict[str, "SerialBus"] = {}
    _buses_lock = threading.Lock()

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.lock = threading.Lock()
        self.ser = None
        # intended = a real port was configured; transient I/O errors then trigger a
        # throttled reopen instead of permanently dropping to dry-run.
        self.intended = bool(port) and serial is not None
        self._next_open = 0.0
        self._open(initial=True)

    def _open(self, initial: bool = False):
        """(Re)open the port, throttled to once/2s. Lets teleop transparently recover
        from a USB hiccup / re-enumeration / brief contention instead of the control
        thread dying on a single 'access denied' (PermissionError) serial error."""
        if not self.intended:
            return
        now = time.perf_counter()
        if not initial and now < self._next_open:
            return
        self._next_open = now + 2.0
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baud, bytesize=8,
                                     parity="N", stopbits=1, timeout=0.05,
                                     write_timeout=0.1)
            if not initial:
                print(f"[serial] {self.port} reopened", flush=True)
        except Exception as e:
            if initial:
                print(f"[serial] could not open {self.port}: {e} -> retrying", flush=True)
            self.ser = None

    def _fail(self, e: Exception):
        """A serial op raised: drop the now-invalid handle so the next call reopens."""
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        print(f"[serial] {self.port} I/O error ({type(e).__name__}: {e}); will reopen", flush=True)

    @classmethod
    def get(cls, port: str, baud: int) -> "SerialBus":
        with cls._buses_lock:
            key = port or "__dryrun__"
            if key not in cls._buses:
                cls._buses[key] = SerialBus(port, baud)
            return cls._buses[key]

    @property
    def is_open(self) -> bool:
        return self.ser is not None

    def write(self, frame: bytes):
        if not self.intended:
            return
        with self.lock:
            if self.ser is None:
                self._open()
                if self.ser is None:
                    return
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
            except (serial.SerialException, OSError) as e:
                self._fail(e)

    def query(self, frame: bytes, expect_bytes: int) -> Optional[bytes]:
        if not self.intended:
            return None
        with self.lock:
            if self.ser is None:
                self._open()
                if self.ser is None:
                    return None
            try:
                self.ser.reset_input_buffer()
                self.ser.write(frame)
                deadline = time.perf_counter() + 0.1
                buf = bytearray()
                while len(buf) < expect_bytes and time.perf_counter() < deadline:
                    chunk = self.ser.read(expect_bytes - len(buf))
                    if chunk:
                        buf += chunk
                return bytes(buf) if len(buf) >= expect_bytes else None
            except (serial.SerialException, OSError) as e:
                self._fail(e)
                return None

    def close(self):
        self.intended = False
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None


def _interp_monotone(x: float, xs: List[float], ys: List[float]) -> float:
    """Monotone cubic (Fritsch-Carlson) interpolation at scalar x.

    xs strictly ascending; clamps flat outside [xs[0], xs[-1]]. With 2 points this
    reduces to linear. Used for the calib curve so a multi-anchor remap is C1-smooth
    (no slope kink, unlike np.interp) while staying monotonic (no overshoot past the
    anchors) -- the kink is what made the old 3-point pinch curve feel abrupt."""
    n = len(xs)
    if n == 0:
        return float(x)
    if n == 1 or x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    h = [xs[i + 1] - xs[i] for i in range(n - 1)]
    d = [(ys[i + 1] - ys[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for i in range(1, n - 1):
        m[i] = 0.0 if d[i - 1] * d[i] <= 0 else (d[i - 1] + d[i]) / 2.0
    for i in range(n - 1):  # Fritsch-Carlson monotonicity limiter
        if d[i] == 0.0:
            m[i] = m[i + 1] = 0.0
        else:
            a, b = m[i] / d[i], m[i + 1] / d[i]
            s = a * a + b * b
            if s > 9.0:
                t = 3.0 / (s ** 0.5)
                m[i], m[i + 1] = t * a * d[i], t * b * d[i]
    for i in range(n - 1):
        if xs[i] <= x <= xs[i + 1]:
            t = (x - xs[i]) / h[i]
            t2, t3 = t * t, t * t * t
            return float((2 * t3 - 3 * t2 + 1) * ys[i]
                         + (t3 - 2 * t2 + t) * h[i] * m[i]
                         + (-2 * t3 + 3 * t2) * ys[i + 1]
                         + (t3 - t2) * h[i] * m[i + 1])
    return float(ys[-1])


@dataclass
class JointMap:
    name: str
    qmin: float
    qmax: float
    out_lo: int
    out_hi: int
    invert: bool
    # Optional measured register-space calibration: list of [raw, real] anchor
    # points, monotonic ascending in both columns and pinned at the ends
    # ([0,0] .. [1000,1000]). Corrects the residual sim->real gap of the
    # underactuated tendon hand (the CAD URDF reaches a pinch at ~half the real
    # motor travel). Applied AFTER the linear qmin/qmax map. None = identity.
    calib: Optional[List[List[float]]] = None
    # Optional: drive this register from ANOTHER register's value instead of this
    # joint's own retargeted angle (index 0..5). Used for the thumb: DexPilot only
    # constrains the thumb TIP, so the 2-DOF thumb's flex is not individually
    # identifiable and comes out erratic -- but in every real pinch/grasp the flex
    # tracks the opposition (measured: flex 679 ~= opp 670). So reg1 (flex) mirrors
    # reg0 (opp). This joint's own `calib` still applies on top (as a gain/curve).
    couple_from: Optional[int] = None

    def apply_calib(self, val: float) -> float:
        if not self.calib:
            return val
        xs = [p[0] for p in self.calib]
        ys = [p[1] for p in self.calib]
        return _interp_monotone(val, xs, ys)

    def invert_calib(self, reg: float) -> float:
        """Inverse of apply_calib (real register -> raw), for the 3D feedback ghost only.
        Sort by the real (output) column so the query axis is ascending even when the
        calib's real column is non-monotonic (e.g. the thumb-flex curve 429->388)."""
        if not self.calib:
            return reg
        pts = sorted(self.calib, key=lambda p: p[1])
        ys = [p[1] for p in pts]   # real values, now ascending -> query axis
        xs = [p[0] for p in pts]   # raw values
        return float(np.interp(reg, ys, xs))


@dataclass
class Feedback:
    force_g: List[int]       # 13 values (regs 18..30)
    joint_angle_deg: List[float]  # 10 values (regs 31..40)
    motor_pos: List[int]     # 6 values (regs 41..46)


class RegisterSmoother:
    """Temporal conditioning of the 0..1000 register vector before it is written.

    Two stages, run every control tick:
      1. spatial EMA            (alpha < 1 smooths; alpha >= 1 is a no-op),
      2. per-register slew limit (|delta| <= max_step per tick; <= 0 disables).
    The slew limit is the master "no abrupt motion" guarantee: even if the optimizer
    or calib produce a step (e.g. DexPilot's pinch projection), the hand eases toward
    it at <= max_step/tick instead of snapping. Shared by the live control loop and
    the offline smoothness check so both see identical conditioning."""

    def __init__(self, alpha: float = 1.0, max_step: float = 0.0):
        self.alpha = float(alpha)
        self.max_step = float(max_step)
        self._ema: Optional[np.ndarray] = None
        self._cmd: Optional[np.ndarray] = None

    def reset(self):
        self._ema = None
        self._cmd = None

    def step(self, target) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        if self._ema is None or self.alpha >= 1.0:
            self._ema = target.copy()
        else:
            self._ema = self._ema + self.alpha * (target - self._ema)
        if self._cmd is None or self.max_step <= 0:
            self._cmd = self._ema.copy()
        else:
            self._cmd = self._cmd + np.clip(self._ema - self._cmd,
                                            -self.max_step, self.max_step)
        return self._cmd


class HandDriver:
    def __init__(self, side: str, slave: int, joints: List[JointMap], bus: SerialBus,
                 deadband: int = 4):
        self.side = side
        self.slave = slave
        self.joints = joints
        self.bus = bus
        self.deadband = deadband
        self._last_written: Optional[List[int]] = None

    @property
    def writing_enabled(self) -> bool:
        return self.bus.is_open

    def qpos_to_registers(self, target_qpos: np.ndarray) -> List[int]:
        # pass 1: each joint's own linear register value (pre-calib, pre-coupling)
        base = []
        for q, jm in zip(target_qpos, self.joints):
            span = (jm.qmax - jm.qmin)
            norm = 0.0 if abs(span) < 1e-9 else (q - jm.qmin) / span
            norm = float(np.clip(norm, 0.0, 1.0))
            if jm.invert:
                norm = 1.0 - norm
            base.append(jm.out_lo + norm * (jm.out_hi - jm.out_lo))
        # pass 2: optional coupling (mirror another register), then calib
        regs = []
        for i, jm in enumerate(self.joints):
            val = base[jm.couple_from] if jm.couple_from is not None else base[i]
            val = jm.apply_calib(val)
            regs.append(int(round(np.clip(val, 0, 1000))))
        return regs

    def registers_to_qpos(self, regs: List[int]) -> np.ndarray:
        """Inverse of qpos_to_registers: 0..1000 registers -> 6 target joint angles.
        Used to drive the actual-hand ghost in the 3D viewer from motor feedback."""
        out = []
        for r, jm in zip(regs, self.joints):
            r = jm.invert_calib(float(r))
            span = jm.out_hi - jm.out_lo
            norm = 0.0 if abs(span) < 1e-9 else (r - jm.out_lo) / span
            norm = float(np.clip(norm, 0.0, 1.0))
            if jm.invert:
                norm = 1.0 - norm
            out.append(jm.qmin + norm * (jm.qmax - jm.qmin))
        return np.array(out, dtype=float)

    def _changed_enough(self, regs: List[int]) -> bool:
        if self._last_written is None:
            return True
        return any(abs(a - b) >= self.deadband for a, b in zip(regs, self._last_written))

    def write_positions(self, regs: List[int], force: bool = False) -> bool:
        if not (force or self._changed_enough(regs)):
            return False
        self.bus.write(build_write_multiple(self.slave, CONTROL_REG_START, regs))
        self._last_written = list(regs)
        return True

    def write_speed(self, value: int):
        self.bus.write(build_write_multiple(self.slave, SPEED_REG_START, [value] * 6))

    def write_force(self, value: int):
        self.bus.write(build_write_multiple(self.slave, FORCE_REG_START, [value] * 6))

    def relax(self):
        self.write_positions([0] * 6, force=True)

    def read_feedback(self) -> Optional[Feedback]:
        # response: slave(1)+func(1)+bytecount(1)+2*N data + crc(2)
        expect = 3 + FEEDBACK_REG_COUNT * 2 + 2
        resp = self.bus.query(build_read_holding(self.slave, FEEDBACK_REG_START,
                                                 FEEDBACK_REG_COUNT), expect)
        if resp is None or len(resp) < expect:
            return None
        if resp[0] != self.slave or resp[1] != 0x03:
            return None
        if modbus_crc16(resp[:-2]) != (resp[-2] | (resp[-1] << 8)):
            return None
        regs = [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(FEEDBACK_REG_COUNT)]
        return Feedback(
            force_g=regs[0:13],
            joint_angle_deg=[r * 0.1 for r in regs[13:23]],
            motor_pos=regs[23:29],
        )


def joints_from_config(cfg_joints: List[dict]) -> List[JointMap]:
    return [JointMap(name=j["name"], qmin=float(j["qmin"]), qmax=float(j["qmax"]),
                     out_lo=int(j["out_lo"]), out_hi=int(j["out_hi"]),
                     invert=bool(j.get("invert", False)),
                     calib=j.get("calib"),
                     couple_from=j.get("couple_from")) for j in cfg_joints]
