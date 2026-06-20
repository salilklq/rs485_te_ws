"""Offline self-test for the retargeting pipeline (no glove, no hardware).

Loads the real hand URDF + dexpilot config, feeds synthetic open / fist / pinch
poses through to_mano_frame -> dex-retargeting -> qpos -> registers, and checks
the mapping behaves sanely:
    * open hand  -> all flex registers near 0
    * full fist  -> all flex registers high
    * thumb pinch-> thumb opposition register (reg0) rises vs open

Run:  conda run -n teleop python tools/test_retarget.py [--hand right|left]
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
from dexhand_teleop import keypoints, hand_driver   # noqa: E402
from dexhand_teleop.retarget import HandRetargeter  # noqa: E402
from tools.fake_streamer import synth_hand          # noqa: E402


def registers(driver, retargeter, kp, is_right):
    mano = keypoints.to_mano_frame(kp, is_right=is_right)
    qmap = retargeter.retarget(mano)
    tq = retargeter.target_qpos(qmap)
    regs = driver.qpos_to_registers(tq)
    return tq, regs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    args = ap.parse_args()
    is_right = args.hand == "right"

    cfg_path = ROOT / "configs" / f"{args.hand}_hand_dexpilot.yml"
    retargeter = HandRetargeter(str(cfg_path), str(ROOT / "assets"))
    print("joint_names :", retargeter.joint_names)
    print("targets     :", retargeter.target_joint_names)
    print("limits      :\n", retargeter.target_limits)

    drive = yaml.safe_load(open(ROOT / "configs" / "drive.yml"))
    joints = hand_driver.joints_from_config(drive[args.hand]["joints"])
    bus = hand_driver.SerialBus.get("", 115200)  # dry-run
    driver = hand_driver.HandDriver(args.hand, drive[args.hand]["slave"], joints, bus)

    poses = {
        "open":   (np.array([0.05, 0.05, 0.05, 0.05, 0.05]), 0.0),
        "fist":   (np.array([0.95, 0.95, 0.95, 0.95, 0.95]), 0.6),
        "pinch":  (np.array([0.6, 0.7, 0.2, 0.2, 0.2]), 0.95),
        "point":  (np.array([0.2, 0.05, 0.9, 0.9, 0.9]), 0.3),
    }
    results = {}
    print("\n%-7s | %-40s | registers reg0..5" % ("pose", "qpos(target, deg)"))
    print("-" * 96)
    for name, (flex, opp) in poses.items():
        # evaluate each pose independently (reset + converge), else the optimizer
        # warm-starts from the previous pose and the table is misleading
        kp = synth_hand(flex, opp, is_right=is_right)
        retargeter.reset()
        for _ in range(25):
            tq, regs = registers(driver, retargeter, kp, is_right)
        results[name] = regs
        deg = ", ".join("%5.1f" % np.degrees(v) for v in tq)
        print("%-7s | %-40s | %s" % (name, deg, regs))

    # sanity checks
    ok = True
    open_flex = np.mean(results["open"][2:])
    fist_flex = np.mean(results["fist"][2:])
    print("\nopen mean finger-reg = %.0f, fist mean finger-reg = %.0f" % (open_flex, fist_flex))
    if not (fist_flex > open_flex + 200):
        print("  [WARN] fist should close fingers much more than open"); ok = False
    if not (results["pinch"][0] > results["open"][0] + 100):
        print("  [WARN] pinch should raise thumb opposition reg0 vs open"); ok = False
    print("\nRESULT:", "PASS" if ok else "CHECK NEEDED (tune scaling/limits/axes)")


if __name__ == "__main__":
    main()
