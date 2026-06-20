"""Lightweight 3D visualizer for the retargeted robot hand (matplotlib + pinocchio FK).

No SAPIEN/GPU needed. Two modes:

  snapshot : render a grid of fixed poses (open/fist/point/pinch) to a PNG so you
             can eyeball that retargeting looks right.
             conda run -n teleop python tools/sim_view.py snapshot --hand right --out sim.png

  live     : open an animated window driven by UDP keypoints (run fake_streamer or
             the real ManusKeypointStreamer first).
             conda run -n teleop python tools/sim_view.py live --hand right
"""
import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import keypoints, protocol           # noqa: E402
from dexhand_teleop.retarget import HandRetargeter        # noqa: E402
from tools.fake_streamer import synth_hand                # noqa: E402

TIP_IDX = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}


def urdf_edges(hand):
    path = ROOT / "assets" / f"{hand}_hand.urdf"
    root = ET.parse(path).getroot()
    edges = []
    links = set()
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        edges.append((p, c))
        links.add(p); links.add(c)
    return edges, sorted(links)


def fk_positions(R, qmap, links):
    r = R.retargeting.optimizer.robot
    full = np.array([qmap[n] for n in R.joint_names])
    r.compute_forward_kinematics(full)
    pos = {}
    for ln in links:
        try:
            pos[ln] = r.get_link_pose(r.get_link_index(ln))[:3, 3]
        except Exception:
            pass
    return pos


def draw_hand(ax, pos, edges, side, title):
    ax.clear()
    for p, c in edges:
        if p in pos and c in pos:
            a, b = pos[p], pos[c]
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], "-", color="#3a7", lw=2)
    pts = np.array(list(pos.values()))
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=8, color="#9cf")
    for f in ["thumb", "index", "middle", "ring", "pinky"]:
        ln = f"hand_{side}_{f}_tip"
        if ln in pos:
            p = pos[ln]
            ax.scatter([p[0]], [p[1]], [p[2]], s=55,
                       color="#ff5a5a" if f == "thumb" else "#ffd166")
    ax.set_title(title, fontsize=9, color="#ccc")
    _equal_aspect(ax, pts)
    ax.set_axis_off()


def _equal_aspect(ax, pts):
    c = pts.mean(0)
    r = max(1e-3, (pts.max(0) - pts.min(0)).max() / 2)
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def pinch_kp(base, target_idx):
    kp = base.copy(); kp[4] = kp[TIP_IDX[target_idx]]; return kp


def snapshot(hand, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    R = HandRetargeter(str(ROOT / "configs" / f"{hand}_hand_dexpilot.yml"), str(ROOT / "assets"))
    edges, links = urdf_edges(hand)
    side = "r" if hand == "right" else "l"
    base = synth_hand(np.array([0.25, 0.15, 0.15, 0.2, 0.25]), 0.3, is_right=(hand == "right"))

    poses = {
        "open": synth_hand(np.array([0.05] * 5), 0.0, hand == "right"),
        "fist": synth_hand(np.array([0.95] * 5), 0.5, hand == "right"),
        "point": synth_hand(np.array([0.2, 0.05, 0.9, 0.9, 0.9]), 0.3, hand == "right"),
        "pinch index": pinch_kp(base, "index"),
        "pinch middle": pinch_kp(base, "middle"),
        "pinch pinky": pinch_kp(base, "pinky"),
    }
    fig = plt.figure(figsize=(13, 8), facecolor="#0a0e17")
    for i, (name, kp) in enumerate(poses.items()):
        mano = keypoints.to_mano_frame(kp, is_right=(hand == "right"))
        R.reset()  # evaluate each snapshot pose independently (no warm-start carryover)
        for _ in range(25):
            qmap = R.retarget(mano)
        pos = fk_positions(R, qmap, links)
        tq = R.target_qpos(qmap)
        regs = [int(round(np.clip((tq[k] - R.target_limits[k, 0]) /
                max(R.target_limits[k, 1] - R.target_limits[k, 0], 1e-6), 0, 1) * 1000)) for k in range(6)]
        ax = fig.add_subplot(2, 3, i + 1, projection="3d", facecolor="#0a0e17")
        draw_hand(ax, pos, edges, side, f"{name}\nreg {regs}")
    fig.suptitle(f"{hand} hand retargeting (red=thumb tip, yellow=finger tips)", color="#9cf")
    fig.tight_layout()
    fig.savefig(out, dpi=110, facecolor="#0a0e17")
    print("wrote", out)


def live(hand, port):
    import matplotlib.pyplot as plt
    from dexhand_teleop.manus_receiver import ManusReceiver
    R = HandRetargeter(str(ROOT / "configs" / f"{hand}_hand_dexpilot.yml"), str(ROOT / "assets"))
    edges, links = urdf_edges(hand)
    side = "r" if hand == "right" else "l"
    sd = protocol.SIDE_RIGHT if hand == "right" else protocol.SIDE_LEFT
    rx = ManusReceiver(port=port); rx.start()
    fig = plt.figure(figsize=(6, 6), facecolor="#0a0e17")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0a0e17")
    print("live view; Ctrl+C to stop")
    try:
        while True:
            frame, age = rx.get_latest(sd)
            if frame is not None and frame.valid and age < 0.5:
                mano = keypoints.to_mano_frame(frame.points, is_right=(hand == "right"))
                qmap = R.retarget(mano)
                pos = fk_positions(R, qmap, links)
                draw_hand(ax, pos, edges, side, f"{hand} live")
                plt.pause(0.03)
            else:
                plt.pause(0.1)
    except KeyboardInterrupt:
        rx.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["snapshot", "live"])
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    ap.add_argument("--out", default="sim.png")
    ap.add_argument("--port", type=int, default=9001)
    args = ap.parse_args()
    if args.mode == "snapshot":
        snapshot(args.hand, args.out)
    else:
        live(args.hand, args.port)


if __name__ == "__main__":
    main()
