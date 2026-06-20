"""Measure the REAL finger curl curve vs the model (drives the hand, reads feedback).

Why this exists
---------------
The pinch diagnostic showed the retarget URDF reaches the thumb-index contact at
roughly HALF the motor travel the real hand needs: in sim the fingertips meet at
flex_reg~344 / index_reg~438, but the real hand only pinches at 679 / 869 -- a
clean ~2x on both fingers. So the retargeting (which faithfully closes the *sim*
tips) emits ~320-445 and the *real* tips stay far apart.

That points at a geometry/coupling mismatch, with two candidate causes:
  (1) finger travel is ~2x too large -- make_retarget_urdf baked A+B = 180 deg
      (proximal 90 + distal 90) per finger, but the real finger may only curl ~90.
  (2) the fingertip frames are too long (tip_scale=2.0) -> fingers over-reach and
      "touch" in sim at low curl.

The real hand REPORTS its joint angles (regs 31..40, unit 0.1 deg) and motor
positions (regs 41..46, 0..1000). This tool sweeps each finger's command register
0 -> 1000 and records both, so we can read the true register->angle curve and tell
the two causes apart:
  * total real curl at reg1000 ~= 90 deg  -> cause (1), shrink A+B in the URDF
  * total real curl at reg1000 ~= 180 deg -> cause (2), shorten the tip frames

This ONLY moves the hand via the normal position/speed/force registers and reads
feedback. It does NOT modify any project file, config, or URDF.

Usage
-----
    conda run -n teleop python tools/calib_finger_curve.py --port COM5 --slave 1

Stop the teleop service first so the COM port is free. Low speed/force by default;
one finger moves at a time (others held open); all registers relaxed to 0 on exit.
Ctrl-C is safe (it relaxes before quitting).
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import hand_driver as hd  # noqa: E402

# command register order (regs 0..5)
CHANNELS = [
    (0, "thumb_opp"),
    (1, "thumb_flex"),
    (2, "index"),
    (3, "middle"),
    (4, "ring"),
    (5, "pinky"),
]

# What the current model assumes, for side-by-side comparison in the summary.
MODEL_RANGE_DEG = {"thumb_opp": 120.0}  # reg0 opposition (qmax 2.0944)
MODEL_FLEX_DEG = 180.0                  # every flex finger: A+B = 90 + 90

# Known ground-truth contact registers (from the panel manual control) for context.
GROUND_TRUTH_PINCH = "thumb_opp=670  thumb_flex=679  index=869  (others 0)"
SIM_CONTACT = "thumb_flex~344  index~438  (model reaches contact here -> the ~2x gap)"


def hold(driver, regs, dwell, rate):
    """Continuously command `regs` for `dwell` seconds so the finger actually moves
    and settles (single-shot writes did not reliably move the hand)."""
    period = 1.0 / rate
    end = time.perf_counter() + dwell
    while time.perf_counter() < end:
        driver.write_positions(regs, force=True)
        time.sleep(period)


def read_settled(driver, tries=4):
    """Read feedback a few times and return the last valid frame."""
    fb = None
    for _ in range(tries):
        f = driver.read_feedback()
        if f is not None:
            fb = f
        time.sleep(0.02)
    return fb


def sweep_channel(driver, reg_index, name, targets, dwell, rate):
    print("\n=== sweep %s (reg%d); others held at 0 ===" % (name, reg_index))
    print("%-7s | %-22s | %s" % ("target", "motor_pos[0..5]", "joint_angle_deg[0..9]"))
    print("-" * 92)
    rows = []
    for t in targets:
        regs = [0, 0, 0, 0, 0, 0]
        regs[reg_index] = t
        hold(driver, regs, dwell, rate)
        fb = read_settled(driver)
        if fb is None:
            print("%-7d | %-22s | (no feedback)" % (t, "?"))
            continue
        mp = fb.motor_pos
        ang = [round(a, 1) for a in fb.joint_angle_deg]
        rows.append((t, mp, fb.joint_angle_deg))
        print("%-7d | %-22s | %s" % (t, str(mp), str(ang)))
    return rows


def summarize(name, rows, move_thresh=5.0):
    """Report which angle slots this channel drives and their measured range."""
    if not rows:
        return
    n_ang = len(rows[0][2])
    lo = [min(r[2][i] for r in rows) for i in range(n_ang)]
    hi = [max(r[2][i] for r in rows) for i in range(n_ang)]
    driven = [i for i in range(n_ang) if (hi[i] - lo[i]) >= move_thresh]
    if not driven:
        print("  %-11s : no joint-angle slot moved >= %.0f deg "
              "(did the command land? check motor_pos tracked target)" % (name, move_thresh))
        return
    parts = ["slot%d %.0f..%.0f (%.0f deg)" % (i, lo[i], hi[i], hi[i] - lo[i]) for i in driven]
    total = sum(hi[i] - lo[i] for i in driven)
    print("  %-11s : %s | total moved = %.0f deg" % (name, "; ".join(parts), total))

    # motor tracking sanity: at the last (max) target, did motor_pos reach it?
    last_t, last_mp, _ = rows[-1]
    print("               motor_pos at target=%d -> %s" % (last_t, str(last_mp)))

    # interpretation hint for flex fingers
    if name != "thumb_opp":
        ratio = total / MODEL_FLEX_DEG if MODEL_FLEX_DEG else 0.0
        verdict = ("~half of model -> CAUSE (1): finger travel A+B is ~2x too large"
                   if total < 0.65 * MODEL_FLEX_DEG else
                   "~matches model 180 deg -> CAUSE (2): tip frames likely too long"
                   if total > 0.85 * MODEL_FLEX_DEG else
                   "between -> mixed; inspect the curve")
        print("               model assumes %.0f deg; real/model = %.2f -> %s"
              % (MODEL_FLEX_DEG, ratio, verdict))
    else:
        print("               model assumes %.0f deg opposition range" % MODEL_RANGE_DEG["thumb_opp"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=300, help="regs 6..11 (0..1000)")
    ap.add_argument("--force", type=int, default=300, help="regs 12..17 (0..1000)")
    ap.add_argument("--step", type=int, default=100, help="register step (default 100 -> 11 points)")
    ap.add_argument("--dwell", type=float, default=1.2, help="seconds to hold each step")
    ap.add_argument("--rate", type=float, default=40.0, help="hold write rate (Hz)")
    ap.add_argument("--channels", default="0,1,2,3,4,5",
                    help="comma list of registers to sweep (default all). pinch = 0,1,2")
    args = ap.parse_args()

    bus = hd.SerialBus.get(args.port, args.baud)
    if not bus.is_open:
        print("ERROR: could not open %s -- is the teleop service still running?" % args.port)
        return
    driver = hd.HandDriver("right", args.slave, [], bus, deadband=0)

    print("opened %s slave %d  speed=%d force=%d  step=%d dwell=%.1fs"
          % (args.port, args.slave, args.speed, args.force, args.step, args.dwell))
    print("ground-truth pinch :", GROUND_TRUTH_PINCH)
    print("sim contact (model):", SIM_CONTACT)
    driver.write_speed(args.speed)
    driver.write_force(args.force)

    targets = list(range(0, 1001, args.step))
    if targets[-1] != 1000:
        targets.append(1000)
    want = {int(c) for c in args.channels.split(",") if c.strip() != ""}

    results = {}
    try:
        for reg_index, name in CHANNELS:
            if reg_index not in want:
                continue
            rows = sweep_channel(driver, reg_index, name, targets, args.dwell, args.rate)
            results[name] = rows
            # relax this finger before moving to the next
            hold(driver, [0] * 6, 0.6, args.rate)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        driver.relax()
        time.sleep(0.3)
        driver.relax()
        print("\nrelaxed all registers to 0.")

    print("\n================= SUMMARY (real register -> joint-angle ranges) =================")
    print("Each finger's joint angles live in joint_angle_deg slots 0..9; this shows which")
    print("slots each register drove and how far. Compare total real curl to the model.\n")
    for _, name in CHANNELS:
        if name in results:
            summarize(name, results[name])
    print("\nNext: if flex fingers show ~half the model's 180 deg, the URDF finger travel")
    print("(A+B) should shrink ~2x; if they match 180 deg, shorten the tip frames instead.")
    print("Re-run the pinch check after the fix: a pinch gesture should output ~%s." % GROUND_TRUTH_PINCH)


if __name__ == "__main__":
    main()
