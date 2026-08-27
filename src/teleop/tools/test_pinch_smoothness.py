"""Offline proof that the pinch closes SMOOTHLY (no abrupt snap) -- no hardware.

Sweeps a continuous  open -> pinch -> open  hand trajectory through the FULL
pipeline (to_mano_frame -> dex-retargeting/DexPilot -> qpos -> qpos_to_registers
-> RegisterSmoother) and measures, for reg0(thumb opp)/reg1(thumb flex)/reg2(index):

  * the largest single-tick register jump -- this is exactly what a "突兀/强拉"
    pinch would look like in the data,
  * raw (pipeline output) vs smoothed (after EMA + slew limit),
  * the settled pinch registers and the settled release registers.

Plus a snap comparison: the same sweep at project_dist = 0.03 (old default) vs the
configured value, to show the DexPilot pinch-snap shrinking.

PASS = every smoothed channel's per-tick jump stays within common.max_step.

Run:  conda run -n teleop python tools/test_pinch_smoothness.py [--hand right|left]
"""
import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # pinocchio(conda)+torch(pip) OpenMP

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import keypoints, hand_driver          # noqa: E402
from dexhand_teleop.retarget import HandRetargeter          # noqa: E402
from tools.fake_streamer import synth_hand                  # noqa: E402

OPEN = (np.array([0.05, 0.05, 0.05, 0.05, 0.05]), 0.0)
PINCH = (np.array([0.6, 0.7, 0.2, 0.2, 0.2]), 0.95)
N_RAMP, N_HOLD = 80, 40


def _traj():
    """open->pinch (ramp+hold) -> open (ramp+hold) as a list of t in [0,1]."""
    up = list(np.linspace(0, 1, N_RAMP)) + [1.0] * N_HOLD
    down = list(np.linspace(1, 0, N_RAMP)) + [0.0] * (N_HOLD // 2)
    return up + down


def _sweep(retargeter, driver, project_dist, smoother=None):
    """Run the trajectory; return list of (t, raw_regs, out_regs)."""
    retargeter.project_dist = project_dist
    retargeter.reset()
    if smoother is not None:
        smoother.reset()
    is_right = "_r_" in driver.joints[0].name or driver.side == "right"
    rows = []
    for t in _traj():
        flex = OPEN[0] + (PINCH[0] - OPEN[0]) * t
        opp = OPEN[1] + (PINCH[1] - OPEN[1]) * t
        kp = synth_hand(flex, opp, is_right=is_right)
        retargeter.retarget(keypoints.to_mano_frame(kp, is_right=is_right))
        raw = np.array(driver.qpos_to_registers(retargeter.target_qpos()), dtype=float)
        out = smoother.step(raw).copy() if smoother is not None else raw
        rows.append((t, raw, out))
    return rows


def _max_jump(seq):
    return float(np.max(np.abs(np.diff(seq)))) if len(seq) > 1 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    args = ap.parse_args()
    is_right = args.hand == "right"

    retargeter = HandRetargeter(str(ROOT / "configs" / f"{args.hand}_hand_dexpilot.yml"),
                                str(ROOT / "assets"))
    drive = yaml.safe_load(open(ROOT / "configs" / "drive.yml"))
    common = drive.get("common", {})
    joints = hand_driver.joints_from_config(drive[args.hand]["joints"])
    bus = hand_driver.SerialBus.get("", 115200)  # dry-run
    driver = hand_driver.HandDriver(args.hand, drive[args.hand]["slave"], joints, bus)
    alpha = float(common.get("smoothing_alpha", 1.0))
    max_step = float(common.get("max_step", 50))
    cfg_project = retargeter.project_dist

    print("hand=%s  project_dist=%.3f  escape_dist=%.3f  smoothing_alpha=%.2f  max_step=%.0f"
          % (args.hand, cfg_project, retargeter.escape_dist, alpha, max_step))

    # --- snap comparison: old 3cm default vs configured ----------------------
    print("\n[snap] raw per-tick maxΔ (no smoother) -- shows DexPilot projection shrinking")
    print("  %-12s | reg0 | reg1 | reg2(index)" % "project_dist")
    for pd in sorted({0.03, round(cfg_project, 4)}, reverse=True):
        rows = _sweep(retargeter, driver, pd, smoother=None)
        j = [_max_jump([r[1][i] for r in rows]) for i in range(3)]
        tag = " (old default)" if abs(pd - 0.03) < 1e-6 else " (configured)"
        print("  %-12.3f | %4.0f | %4.0f | %4.0f%s" % (pd, j[0], j[1], j[2], tag))

    # --- full pipeline with the temporal smoother ----------------------------
    smoother = hand_driver.RegisterSmoother(alpha, max_step)
    rows = _sweep(retargeter, driver, cfg_project, smoother=smoother)

    print("\n[smooth] per-tick maxΔ raw vs smoothed (configured project_dist)")
    print("  %-16s | %12s | %14s" % ("channel", "raw maxΔ/tick", "smoothed maxΔ/tick"))
    names = ["reg0 thumb_opp", "reg1 thumb_flex", "reg2 index"]
    ok = True
    for i in range(3):
        rj = _max_jump([r[1][i] for r in rows])
        sj = _max_jump([r[2][i] for r in rows])
        bad = sj > max_step + 1.5
        ok = ok and not bad
        print("  %-16s | %12.1f | %14.1f%s"
              % (names[i], rj, sj, "  <-- EXCEEDS max_step!" if bad else ""))

    pinch = rows[N_RAMP + N_HOLD - 1][2]
    rel = rows[-1][2]
    print("\nsettled @ full pinch (reg0/1/2): %.0f / %.0f / %.0f" % (pinch[0], pinch[1], pinch[2]))
    print("settled @ release    (reg0/1/2): %.0f / %.0f / %.0f" % (rel[0], rel[1], rel[2]))

    print("\n   t   | reg0 raw->sm | reg1 raw->sm | reg2 raw->sm")
    for k in range(0, len(rows), max(1, len(rows) // 16)):
        t, raw, sm = rows[k]
        print("  %.2f  |  %4.0f %4.0f   |  %4.0f %4.0f   |  %4.0f %4.0f"
              % (t, raw[0], sm[0], raw[1], sm[1], raw[2], sm[2]))

    print("\nRESULT:", "PASS (smoothed motion bounded by slew limit)" if ok
          else "CHECK (a smoothed channel exceeded max_step)")


if __name__ == "__main__":
    main()
