"""Diagnostic: does the retargeted thumb-FLEX register actually respond to the
human thumb curling, or is it stuck (pinch driven only by opposition)?

Sweeps the synthetic human thumb flex 0..1 at a fixed opposition, and separately
sweeps opposition, printing the RAW reg0/reg1 (no calib) + robot thumb-tip xyz.
"""
import os, sys
from pathlib import Path
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import keypoints, hand_driver
from dexhand_teleop.retarget import HandRetargeter
from tools.fake_streamer import synth_hand

is_right = True
R = HandRetargeter(str(ROOT / "configs" / "right_hand_dexpilot.yml"), str(ROOT / "assets"))
import yaml
drive = yaml.safe_load(open(ROOT / "configs" / "drive.yml"))
# strip calib so we see RAW optimizer output
for j in drive["right"]["joints"]:
    j.pop("calib", None)
joints = hand_driver.joints_from_config(drive["right"]["joints"])
driver = hand_driver.HandDriver("right", 1, joints, hand_driver.SerialBus.get("", 115200))


def run(flex_vec, opp):
    kp = synth_hand(np.array(flex_vec), opp, is_right=is_right)
    mano = keypoints.to_mano_frame(kp, is_right=is_right)
    R.reset()
    regs = None
    for _ in range(25):
        qmap = R.retarget(mano)
        regs = driver.qpos_to_registers(R.target_qpos(qmap))
    r = R.retargeting.optimizer.robot
    full = np.array([qmap[n] for n in R.joint_names])
    r.compute_forward_kinematics(full)
    tp = r.get_link_pose(r.get_link_index("hand_r_thumb_tip"))[:3, 3]
    return regs, tp


print("== sweep human THUMB FLEX (opp fixed 0.9), others 0.2 ==")
print("flexIn | reg0 opp | reg1 flex | thumb_tip xyz (mm)")
for tf in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    regs, tp = run([tf, 0.2, 0.2, 0.2, 0.2], 0.9)
    print(" %.1f   | %8d | %9d | %s" % (tf, regs[0], regs[1], np.round(tp*1000,1)))

print("\n== sweep human OPPOSITION (thumb flex fixed 0.5), others 0.2 ==")
print("oppIn  | reg0 opp | reg1 flex | thumb_tip xyz (mm)")
for op in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]:
    regs, tp = run([0.5, 0.2, 0.2, 0.2, 0.2], op)
    print(" %.1f   | %8d | %9d | %s" % (op, regs[0], regs[1], np.round(tp*1000,1)))
