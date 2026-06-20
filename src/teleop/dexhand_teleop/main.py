"""Dual-hand teleop orchestrator.

    MANUS keypoints (UDP) -> dex-retargeting (DexPilot) -> qpos
    -> register mapping -> RS485 (Modbus-RTU) + live tuning panel.

Run:
    conda run -n teleop python -m dexhand_teleop.main --config configs/drive.yml
"""
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # pinocchio(conda)+torch(pip) OpenMP

import argparse
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import yaml

from . import keypoints, protocol
from .hand_driver import HandDriver, SerialBus, joints_from_config
from .manus_receiver import ManusReceiver
from .retarget import HandRetargeter

SIDE_OF_NAME = {"right": protocol.SIDE_RIGHT, "left": protocol.SIDE_LEFT}


class HandState:
    """Live, panel-visible state for one hand (guarded by `lock`)."""

    def __init__(self, name: str):
        self.name = name
        self.lock = threading.Lock()
        self.connected = False
        self.age = float("inf")
        self.raw_points = np.zeros((21, 3))
        self.mano_points = np.zeros((21, 3))
        self.qpos: Dict[str, float] = {}
        self.target_qpos = np.zeros(6)
        self.registers = [0] * 6
        self.feedback = None
        self.write_enabled = False
        self.rate = 0.0
        self.q_cmd = None      # full pinocchio qpos (commanded), for 3D viz
        self.q_actual = None   # full pinocchio qpos from motor feedback, for 3D ghost


class HandWorker:
    def __init__(self, name: str, cfg: dict, drive: dict, root: Path,
                 receiver: ManusReceiver, common: dict, udp_cfg: dict):
        self.name = name
        self.side = SIDE_OF_NAME[name]
        self.receiver = receiver
        self.common = common
        self.position_scale = float(udp_cfg.get("position_scale", 1.0))
        self.state = HandState(name)
        self.running = False

        self.retargeter = HandRetargeter(
            str(root / "configs" / f"{name}_hand_dexpilot.yml"), str(root / "assets"))
        joints = joints_from_config(drive["joints"])
        bus = SerialBus.get(drive.get("port", ""), int(common.get("baud", 115200)))
        self.driver = HandDriver(name, int(drive["slave"]), joints, bus,
                                 deadband=int(common.get("deadband", 4)))
        self.state.write_enabled = self.driver.writing_enabled

        # register-space extra smoothing
        self.smoothing_alpha = float(common.get("smoothing_alpha", 1.0))
        self._smoothed: Optional[np.ndarray] = None
        self._calib: Dict[str, np.ndarray] = {}
        self._fb_decimate = 0

        # startup speed/force
        if self.driver.writing_enabled:
            if int(common.get("speed", -1)) >= 0:
                self.driver.write_speed(int(common["speed"]))
            if int(common.get("force", -1)) >= 0:
                self.driver.write_force(int(common["force"]))

    # -- control loop ---------------------------------------------------------
    def run(self, stop_event: threading.Event):
        self.running = True
        rate = float(self.common.get("rate_hz", 60))
        period = 1.0 / rate
        read_fb = bool(self.common.get("read_feedback", True))
        next_t = time.perf_counter()
        last = next_t
        while not stop_event.is_set():
            frame, age = self.receiver.get_latest(self.side)
            now = time.perf_counter()
            if frame is not None and frame.valid and age < 0.5:
                mano = keypoints.to_mano_frame(frame.points, is_right=(self.side == protocol.SIDE_RIGHT),
                                               position_scale=self.position_scale)
                qmap = self.retargeter.retarget(mano)
                tq = self.retargeter.target_qpos(qmap)
                regs = np.array(self.driver.qpos_to_registers(tq), dtype=float)
                if self._smoothed is None or self.smoothing_alpha >= 1.0:
                    self._smoothed = regs
                else:
                    self._smoothed = self._smoothed + self.smoothing_alpha * (regs - self._smoothed)
                regs_i = [int(round(v)) for v in self._smoothed]
                if self.driver.writing_enabled:
                    self.driver.write_positions(regs_i)
                with self.state.lock:
                    self.state.connected = True
                    self.state.age = age
                    self.state.raw_points = frame.points
                    self.state.mano_points = mano
                    self.state.qpos = qmap
                    self.state.target_qpos = tq
                    self.state.registers = regs_i
                    self.state.rate = 1.0 / max(now - last, 1e-6)
                    self.state.q_cmd = np.array(
                        [qmap[n] for n in self.retargeter.joint_names], dtype=float)
            else:
                with self.state.lock:
                    self.state.connected = False
                    self.state.age = age

            if read_fb and self.driver.writing_enabled:
                self._fb_decimate = (self._fb_decimate + 1) % 5  # ~rate/5
                if self._fb_decimate == 0:
                    fb = self.driver.read_feedback()
                    if fb is not None:
                        qa = self.retargeter.expand_targets(
                            self.driver.registers_to_qpos(fb.motor_pos))
                        with self.state.lock:
                            self.state.feedback = fb
                            self.state.q_actual = qa
            last = now
            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
        self.running = False

    # -- panel hooks ----------------------------------------------------------
    def set_param(self, key: str, value):
        if key == "scaling":
            self.retargeter.scaling = float(value)
        elif key == "low_pass_alpha":
            self.retargeter.low_pass_alpha = float(value)
        elif key == "smoothing_alpha":
            self.smoothing_alpha = float(np.clip(value, 0.0, 1.0))
        elif key == "deadband":
            self.driver.deadband = int(value)
        elif key == "speed" and self.driver.writing_enabled:
            self.driver.write_speed(int(value))
        elif key == "force" and self.driver.writing_enabled:
            self.driver.write_force(int(value))
        elif key == "position_scale":
            self.position_scale = float(value)

    def set_joint_param(self, idx: int, field: str, value):
        jm = self.driver.joints[idx]
        if field == "out_lo":
            jm.out_lo = int(value)
        elif field == "out_hi":
            jm.out_hi = int(value)
        elif field == "invert":
            jm.invert = bool(value)
        elif field == "qmin":
            jm.qmin = float(value)
        elif field == "qmax":
            jm.qmax = float(value)

    def capture(self, pose: str):
        """Capture current target qpos as 'open' or 'fist' to auto-set qmin/qmax."""
        with self.state.lock:
            tq = np.array(self.state.target_qpos, dtype=float)
        self._calib[pose] = tq
        if "open" in self._calib and "fist" in self._calib:
            lo = self._calib["open"]
            hi = self._calib["fist"]
            for i, jm in enumerate(self.driver.joints):
                a, b = float(lo[i]), float(hi[i])
                if abs(b - a) > 1e-3:
                    jm.qmin, jm.qmax = (a, b) if b > a else (b, a)
                    jm.invert = b < a
        return {"captured": pose, "have": list(self._calib.keys())}

    def command(self, cmd: str):
        if cmd == "relax" and self.driver.writing_enabled:
            self.driver.relax()
        elif cmd == "reset":
            self.retargeter.reset()
            self._smoothed = None


class TeleopService:
    def __init__(self, config_path: str, viz: bool = True):
        self.root = Path(config_path).resolve().parent.parent
        self.drive = yaml.safe_load(open(config_path))
        self.common = self.drive.get("common", {})
        udp_cfg = self.drive.get("udp", {})
        self.receiver = ManusReceiver(udp_cfg.get("host", "127.0.0.1"),
                                      int(udp_cfg.get("port", 9001)))
        self.workers: Dict[str, HandWorker] = {}
        for name in ("right", "left"):
            hand_cfg = self.drive.get(name, {})
            if hand_cfg.get("enabled", False):
                self.workers[name] = HandWorker(name, {}, hand_cfg, self.root,
                                                self.receiver, self.common, udp_cfg)
        self._threads = []
        self._stop = threading.Event()

        # 3D meshcat scene (commanded + actual ghost). Best-effort; never blocks teleop.
        self.scene = None
        self.meshcat_url = None
        if viz and self.workers:
            try:
                from .viz_meshcat import HandSceneViz
                hands = {name: (str(self.root / "assets" / f"{name}_hand.urdf"),
                                str(self.root / "assets")) for name in self.workers}
                self.scene = HandSceneViz(hands)
                self.meshcat_url = self.scene.url
            except Exception as e:
                print("3D viz disabled:", e)

    def start(self):
        self.receiver.start()
        for w in self.workers.values():
            t = threading.Thread(target=w.run, args=(self._stop,), daemon=True,
                                 name=f"ctrl-{w.name}")
            t.start()
            self._threads.append(t)
        if self.scene is not None:
            t = threading.Thread(target=self._viz_loop, daemon=True, name="viz")
            t.start()
            self._threads.append(t)

    def _viz_loop(self):
        while not self._stop.is_set():
            for name, w in self.workers.items():
                with w.state.lock:
                    qc = None if w.state.q_cmd is None else np.array(w.state.q_cmd)
                    qa = None if w.state.q_actual is None else np.array(w.state.q_actual)
                try:
                    self.scene.update(name, qc, qa)
                except Exception:
                    pass
            self._stop.wait(1.0 / 30.0)

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        if self.common.get("relax_on_exit", True):
            for w in self.workers.values():
                if w.driver.writing_enabled:
                    w.driver.relax()
        self.receiver.stop()
        if self.scene is not None:
            self.scene.close()

    # -- snapshot for panel ---------------------------------------------------
    def snapshot(self) -> dict:
        out = {"packets": self.receiver.packet_count, "meshcat_url": self.meshcat_url,
               "hands": {}}
        for name, w in self.workers.items():
            s = w.state
            with s.lock:
                fb = s.feedback
                out["hands"][name] = {
                    "connected": s.connected,
                    "age": None if s.age == float("inf") else round(s.age, 3),
                    "rate": round(s.rate, 1),
                    "write_enabled": w.driver.writing_enabled,
                    "registers": list(s.registers),
                    "target_qpos_deg": [round(float(np.degrees(v)), 1) for v in s.target_qpos],
                    "target_joints": w.retargeter.target_joint_names,
                    "raw": s.raw_points.round(4).tolist(),
                    "mano": s.mano_points.round(4).tolist(),
                    "scaling": w.retargeter.scaling,
                    "low_pass_alpha": w.retargeter.low_pass_alpha,
                    "smoothing_alpha": w.smoothing_alpha,
                    "deadband": w.driver.deadband,
                    "joints": [
                        {"name": jm.name, "qmin": round(jm.qmin, 4), "qmax": round(jm.qmax, 4),
                         "out_lo": jm.out_lo, "out_hi": jm.out_hi, "invert": jm.invert}
                        for jm in w.driver.joints
                    ],
                    "feedback": None if fb is None else {
                        "force_g": fb.force_g, "joint_angle_deg": fb.joint_angle_deg,
                        "motor_pos": fb.motor_pos},
                }
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent.parent / "configs" / "drive.yml"))
    ap.add_argument("--panel-port", type=int, default=8090)
    ap.add_argument("--no-panel", action="store_true")
    ap.add_argument("--no-viz", action="store_true", help="disable the 3D meshcat view")
    args = ap.parse_args()

    service = TeleopService(args.config, viz=not args.no_viz)
    service.start()
    print(f"teleop started; hands={list(service.workers)}", flush=True)
    if service.meshcat_url:
        print(f"3D view (meshcat): {service.meshcat_url}", flush=True)

    panel_thread = None
    if not args.no_panel:
        from .panel import run_panel
        panel_thread = threading.Thread(target=run_panel, args=(service, args.panel_port),
                                        daemon=True, name="panel")
        panel_thread.start()
        print(f"tuning panel: http://127.0.0.1:{args.panel_port}/")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print("stopping...")
        service.stop()


if __name__ == "__main__":
    main()
