"""Convert the SolidWorks-exported LZ-SG002 hand URDF into a *retarget-ready* URDF
for dex-retargeting.

The exported URDF models every finger with 2 phalanx joints (thumb: 1 opposition
+ 2 flex), i.e. 11 revolute joints, while the real hand has only 6 actuators
(reg 0..5). dex-retargeting handles this the same way the reference Inspire-hand
URDF does:

  * the *proximal* joint of each finger is an optimized *target* joint,
  * the *distal* joint(s) are coupled to the proximal one with a <mimic> tag, so
    the optimizer effectively sees a 6-DOF hand,
  * a fixed *_tip frame is added at each fingertip for the DexPilot fingertip
    vectors (finger_tip_link_names).

The 6 target joints become, in this exact order, registers 0..5:

  reg0 thumb opposition/yaw  -> thumb chain joint #1 (the ~120 deg axis)
  reg1 thumb flexion/pitch   -> thumb chain joint #2
  reg2 index  flexion        -> index  proximal joint
  reg3 middle flexion        -> middle proximal joint
  reg4 ring   flexion        -> ring   proximal joint
  reg5 pinky  flexion        -> pinky  proximal joint

Joint names differ between the L/R exports (e.g. the right thumb's distal joint
is literally named ``hand_r_thumb_Link3``), so fingers and their proximal/distal
order are detected *structurally* from the child-link names, never hard-coded.

Usage:
    python make_retarget_urdf.py INPUT.urdf OUTPUT.urdf [--tip-scale 2.0]
                                  [--mimic-mult 1.0] [--no-fix-mesh-paths]

The tool prints the discovered target joints + limits so they can be pasted into
the dexpilot YAML / drive config.
"""
import argparse
import copy
import os
import re
import xml.etree.ElementTree as ET

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


def _finger_of(name: str):
    low = name.lower()
    for f in FINGERS:
        if f in low:
            return f
    return None


def _link_order_key(child_link: str) -> int:
    """Order joints within a finger by the trailing number of the child link
    (Link1 < Link2 < Link3). Falls back to 0 when no number is present."""
    m = re.search(r"(\d+)\s*$", child_link)
    return int(m.group(1)) if m else 0


def _vec3(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return list(default)
    return [float(x) for x in text.split()]


def build(input_path: str, output_path: str, tip_scale: float,
          mimic_mult: float, fix_mesh_paths: bool, coupling: str = "sequential"):
    tree = ET.parse(input_path)
    robot = tree.getroot()

    links = {l.get("name"): l for l in robot.findall("link")}
    joints = robot.findall("joint")

    # Group revolute joints by finger and order proximal -> distal.
    by_finger = {f: [] for f in FINGERS}
    for j in joints:
        if j.get("type") != "revolute":
            continue
        child = j.find("child").get("link")
        f = _finger_of(j.get("name")) or _finger_of(child)
        if f is None:
            continue
        by_finger[f].append(j)
    for f in FINGERS:
        by_finger[f].sort(key=lambda j: _link_order_key(j.find("child").get("link")))

    # ---- Decide target vs mimic joints --------------------------------------
    # thumb: [yaw=target, pitch=target, distal=mimic(pitch)]
    # other: [proximal=target, distal=mimic(proximal)]
    target_joints = []          # ordered -> registers 0..5
    mimic_pairs = []            # (mimic_joint_elem, source_joint_name)

    thumb = by_finger["thumb"]
    if len(thumb) < 2:
        raise SystemExit("thumb must have >=2 joints; got %d" % len(thumb))
    yaw, pitch = thumb[0], thumb[1]
    target_joints += [yaw, pitch]
    for distal in thumb[2:]:
        mimic_pairs.append((distal, pitch.get("name")))

    for f in ["index", "middle", "ring", "pinky"]:
        chain = by_finger[f]
        if not chain:
            raise SystemExit("finger %s has no joints" % f)
        proximal = chain[0]
        target_joints.append(proximal)
        for distal in chain[1:]:
            mimic_pairs.append((distal, proximal.get("name")))

    # ---- Apply <mimic> tags -------------------------------------------------
    # The <mimic> tag (linear) is kept only so dex-retargeting discovers the
    # source/mimic pairs; the runtime swaps in a sequential coupling adaptor.
    for mimic_joint, source_name in mimic_pairs:
        for old in mimic_joint.findall("mimic"):
            mimic_joint.remove(old)
        ET.SubElement(mimic_joint, "mimic", {
            "joint": source_name,
            "multiplier": "%.6g" % mimic_mult,
            "offset": "0",
        })

    # ---- Sequential (underactuated tendon) coupling -------------------------
    # Real finger: tendon bends proximal first, then distal. We model the actuator
    # coordinate theta as the proximal/source joint, extending its upper limit to
    # A+B (= max travel) so the optimizer can drive both segments in sequence
    # (q_prox=min(theta,A), q_dist=clip(theta-A,0,B), done by SequentialCouplingAdaptor).
    joint_by_name = {j.get("name"): j for j in joints}
    if coupling == "sequential":
        for mimic_joint, source_name in mimic_pairs:
            src = joint_by_name.get(source_name)
            src_lim = src.find("limit") if src is not None else None
            dist_lim = mimic_joint.find("limit")
            if src_lim is None or dist_lim is None:
                continue
            a = float(src_lim.get("upper", "0"))
            b = float(dist_lim.get("upper", "0"))
            src_lim.set("upper", "%.6g" % (a + b))

    # ---- Add fingertip frames ----------------------------------------------
    # tip placed at (tip_scale * distal-link inertial COM) in the distal frame.
    distal_link_of = {}
    thumb_distal = thumb[-1].find("child").get("link")
    distal_link_of["thumb"] = thumb_distal
    for f in ["index", "middle", "ring", "pinky"]:
        distal_link_of[f] = by_finger[f][-1].find("child").get("link")

    tip_link_names = {}
    for f in FINGERS:
        distal_link = distal_link_of[f]
        link_el = links[distal_link]
        inertial = link_el.find("inertial")
        com = (0.0, 0.0, 0.0)
        if inertial is not None and inertial.find("origin") is not None:
            com = _vec3(inertial.find("origin").get("xyz"))
        tip_xyz = [tip_scale * c for c in com]
        # guard against a zero COM (e.g. pinky Link1 had all-zero inertia)
        if max(abs(v) for v in tip_xyz) < 1e-4:
            tip_xyz = [0.0, 0.03, 0.0]

        side = "r" if "_r_" in distal_link or distal_link.startswith("hand_r") else "l"
        tip_name = "hand_%s_%s_tip" % (side, f)
        tip_link_names[f] = tip_name

        ET.SubElement(robot, "link", {"name": tip_name})
        tip_joint = ET.SubElement(robot, "joint",
                                  {"name": tip_name + "_joint", "type": "fixed"})
        ET.SubElement(tip_joint, "origin",
                      {"xyz": "%.6g %.6g %.6g" % tuple(tip_xyz), "rpy": "0 0 0"})
        ET.SubElement(tip_joint, "parent", {"link": distal_link})
        ET.SubElement(tip_joint, "child", {"link": tip_name})

    # ---- Mesh paths: copy STLs next to the output URDF, use relative paths ---
    # (meshes are NOT needed for retargeting, only for optional visualization;
    #  relative paths keep yourdfpy/pinocchio quiet and make viz work.)
    if fix_mesh_paths:
        import shutil
        src_dir = os.path.abspath(
            os.path.join(os.path.dirname(input_path), "..", "meshes"))
        out_mesh_dir = os.path.join(os.path.dirname(os.path.abspath(output_path)), "meshes")
        if os.path.isdir(src_dir):
            os.makedirs(out_mesh_dir, exist_ok=True)
            for mesh in robot.iter("mesh"):
                fn = mesh.get("filename", "")
                m = re.search(r"([^/\\]+\.STL)$", fn, re.IGNORECASE)
                if not m:
                    continue
                name = m.group(1)
                src = os.path.join(src_dir, name)
                if os.path.isfile(src):
                    shutil.copy2(src, os.path.join(out_mesh_dir, name))
                mesh.set("filename", "meshes/" + name)

    # ---- Write --------------------------------------------------------------
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    base_link = None
    for j in joints:
        if _finger_of(j.find("child").get("link")) is None and j.get("type") == "revolute":
            base_link = j.find("parent").get("link")
            break
    if base_link is None:
        # all chains hang off the same base link
        base_link = target_joints[0].find("parent").get("link")

    print("Wrote", output_path)
    print("wrist_link_name :", base_link)
    print("finger_tip_link_names :", [tip_link_names[f] for f in FINGERS])
    print("target_joint_names (reg0..5):")
    for i, j in enumerate(target_joints):
        lim = j.find("limit")
        lo = lim.get("lower") if lim is not None else "?"
        up = lim.get("upper") if lim is not None else "?"
        print("  reg%d  %-26s [%s, %s]" % (i, j.get("name"), lo, up))
    print("mimic joints:")
    for mj, src in mimic_pairs:
        print("  %-26s mimic %s" % (mj.get("name"), src))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--tip-scale", type=float, default=2.0,
                    help="tip offset = tip_scale * distal-link COM (default 2.0)")
    ap.add_argument("--mimic-mult", type=float, default=1.0,
                    help="distal/proximal coupling ratio (default 1.0)")
    ap.add_argument("--no-fix-mesh-paths", action="store_true")
    ap.add_argument("--coupling", choices=["sequential", "proportional"], default="sequential",
                    help="finger transmission model (default sequential = underactuated tendon)")
    args = ap.parse_args()
    build(args.input, args.output, args.tip_scale, args.mimic_mult,
          not args.no_fix_mesh_paths, args.coupling)


if __name__ == "__main__":
    main()
