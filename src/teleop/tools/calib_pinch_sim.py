"""Calibrate the URDF fingertip length so a *perfect* pinch in sim outputs the
real hand's measured contact registers.

Why
---
The real hand pinches thumb<->index at registers (opp=670, flex=679, index=869)
[measured from the panel]. With tip_scale=2.0 the retarget URDF reaches the same
tip-coincident pinch at roughly HALF that travel -- the fingertip frames stick out
~2x too far, so the sim tips meet while the real tips are still apart. Result:
open/fist look fine (clamped to the rails) but the mid-range and pinch are wrong.

This regenerates the URDF at a candidate --tip-scale, then for a *perfect* pinch
(human thumb tip placed exactly on a fingertip) runs the EXACT live driver path
  to_mano_frame -> retarget -> target_qpos (theta) -> driver.qpos_to_registers
and prints all 6 registers, so we can pick the tip_scale whose index/middle pinch
lands on the ground truth. Sweep with --sweep, or set one value with --tip-scale.

    conda run -n teleop python tools/calib_pinch_sim.py --hand right --sweep 2.0,1.5,1.2,1.0,0.8
    conda run -n teleop python tools/calib_pinch_sim.py --hand right --tip-scale 1.0
"""
import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent          # src/teleop
REPO = ROOT.parent.parent                              # rs485_te_ws
sys.path.insert(0, str(ROOT))

from dexhand_teleop import keypoints, hand_driver        # noqa: E402
from dexhand_teleop.retarget import HandRetargeter        # noqa: E402
from tools.fake_streamer import synth_hand                # noqa: E402
import tools.make_retarget_urdf as mk                     # noqa: E402

SRC_URDF = {
    "right": REPO / "LZ-SG002-URDF.SLDASM" / "LZ-SG002-URDF-R.SLDASM" / "urdf" / "LZ-SG002-URDF-R.SLDASM.urdf",
    "left":  REPO / "LZ-SG002-URDF.SLDASM" / "LG-SG002-URDF-L.SLDASM" / "urdf" / "LG-SG002-URDF-L.SLDASM.urdf",
}
OUT_URDF = {"right": ROOT / "assets" / "right_hand.urdf", "left": ROOT / "assets" / "left_hand.urdf"}
TIP_IDX = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}
GROUND_TRUTH = {"opp": 670, "flex": 679, "index": 869}   # measured real thumb<->index contact


def regen(hand, tip_scale, mimic_mult=1.0, coupling="sequential"):
    mk.build(str(SRC_URDF[hand]), str(OUT_URDF[hand]), tip_scale, mimic_mult,
             fix_mesh_paths=False, coupling=coupling)


def perfect_pinch_regs(retargeter, driver, target, is_right):
    """Full 6 registers (live driver path) for a perfect thumb<->`target` pinch,
    plus the robot thumb-tip <-> target-tip distance (mm)."""
    base = synth_hand(np.array([0.25, 0.15, 0.15, 0.2, 0.25]), 0.3, is_right=is_right)
    kp = base.copy()
    kp[4] = kp[TIP_IDX[target]]                 # thumb tip ON target fingertip
    mano = keypoints.to_mano_frame(kp, is_right=is_right)
    retargeter.reset()
    regs = None
    for _ in range(25):
        qmap = retargeter.retarget(mano)
        tq = retargeter.target_qpos(qmap)
        regs = driver.qpos_to_registers(tq)
    # robot tip distance after convergence
    r = retargeter.retargeting.optimizer.robot
    full = np.array([qmap[n] for n in retargeter.joint_names])
    r.compute_forward_kinematics(full)
    side = "r" if is_right else "l"
    tp = r.get_link_pose(r.get_link_index(f"hand_{side}_thumb_tip"))[:3, 3]
    fp = r.get_link_pose(r.get_link_index(f"hand_{side}_{target}_tip"))[:3, 3]
    dist = float(np.linalg.norm(tp - fp)) * 1000.0
    return regs, dist


def evaluate(hand, tip_scale):
    is_right = hand == "right"
    regen(hand, tip_scale)
    cfg = ROOT / "configs" / f"{hand}_hand_dexpilot.yml"
    retargeter = HandRetargeter(str(cfg), str(ROOT / "assets"))
    drive = yaml.safe_load(open(ROOT / "configs" / "drive.yml"))
    joints = hand_driver.joints_from_config(drive[hand]["joints"])
    bus = hand_driver.SerialBus.get("", 115200)
    driver = hand_driver.HandDriver(hand, drive[hand]["slave"], joints, bus)

    print(f"\n##### tip_scale = {tip_scale}  ({hand}) "
          f"#####   GT index-pinch -> opp={GROUND_TRUTH['opp']} flex={GROUND_TRUTH['flex']} index={GROUND_TRUTH['index']}")
    print("%-7s | reg0 opp | reg1 flex | reg2 idx | reg3 mid | reg4 rng | reg5 pky | tip dist" % "target")
    print("-" * 92)
    for target in ("index", "middle"):
        regs, dist = perfect_pinch_regs(retargeter, driver, target, is_right)
        print("%-7s | %8d | %9d | %8d | %8d | %8d | %8d | %5.1f mm"
              % (target, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], dist))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    ap.add_argument("--tip-scale", type=float, default=None)
    ap.add_argument("--sweep", default=None, help="comma list of tip_scale values")
    args = ap.parse_args()

    if args.sweep:
        scales = [float(s) for s in args.sweep.split(",") if s.strip()]
    elif args.tip_scale is not None:
        scales = [args.tip_scale]
    else:
        scales = [2.0, 1.5, 1.2, 1.0, 0.8]

    for s in scales:
        evaluate(args.hand, s)

    print("\nNOTE: this overwrote assets/%s_hand.urdf at tip_scale=%s. Regenerate the"
          " final chosen value (both hands) before using." % (args.hand, scales[-1]))


if __name__ == "__main__":
    main()
