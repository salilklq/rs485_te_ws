"""Check which fingers the robot thumb can actually pinch (thumb opposition reach).

For each target finger we synthesise a human pinch (thumb TIP placed exactly on
that finger's TIP -- DexPilot only uses wrist + 5 fingertips), retarget, run FK on
the robot URDF, and measure the resulting robot thumb-tip -> finger-tip distance.
A small distance = the robot can physically pinch that finger.

    conda run -n teleop python tools/test_pinch.py --hand right
"""
import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")  # pinocchio(conda)+torch(pip) OpenMP

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import keypoints                 # noqa: E402
from dexhand_teleop.retarget import HandRetargeter   # noqa: E402
from tools.fake_streamer import synth_hand           # noqa: E402

TIP_IDX = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}


def robot_tip_positions(retargeter, qmap):
    r = retargeter.retargeting.optimizer.robot
    full = np.array([qmap[n] for n in retargeter.joint_names])  # pin order
    r.compute_forward_kinematics(full)
    side = "r" if retargeter.target_joint_names[0].startswith("hand_r") else "l"
    names = {f: f"hand_{side}_{f}_tip" for f in ["thumb", "index", "middle", "ring", "pinky"]}
    pos = {}
    for f, ln in names.items():
        pose = r.get_link_pose(r.get_link_index(ln))
        pos[f] = pose[:3, 3]
    return pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    args = ap.parse_args()
    is_right = args.hand == "right"
    R = HandRetargeter(str(ROOT / "configs" / f"{args.hand}_hand_dexpilot.yml"), str(ROOT / "assets"))

    base = synth_hand(np.array([0.25, 0.15, 0.15, 0.2, 0.25]), 0.3, is_right=is_right)
    j_opp, j_flex = R.target_joint_names[0], R.target_joint_names[1]
    print("target  | thumb dist | reg0 opp | reg1 flex | targets correct finger?")
    print("-" * 74)
    for target, ti in TIP_IDX.items():
        kp = base.copy()
        kp[4] = kp[ti]  # human thumb tip ON the target fingertip (perfect pinch)
        mano = keypoints.to_mano_frame(kp, is_right=is_right)
        for _ in range(8):  # warm up optimizer
            qmap = R.retarget(mano)
        pos = robot_tip_positions(R, qmap)
        d_target = float(np.linalg.norm(pos["thumb"] - pos[target])) * 1000.0  # mm
        dists = {f: float(np.linalg.norm(pos["thumb"] - pos[f])) * 1000.0 for f in TIP_IDX}
        nearest = min(dists, key=dists.get)
        reg0 = int(round(np.clip(qmap[j_opp] / 2.0944, 0, 1) * 1000))
        reg1 = int(round(np.clip(qmap[j_flex] / 1.5708, 0, 1) * 1000))
        print("%-7s | %7.1f mm | %8d | %9d | nearest=%s %s" %
              (target, d_target, reg0, reg1, nearest, "<-OK" if nearest == target else "(out of reach)"))


if __name__ == "__main__":
    main()
