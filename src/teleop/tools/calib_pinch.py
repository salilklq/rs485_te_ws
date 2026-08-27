"""Find the hand's thumb<->index CONTACT register -> the calib contact anchor.

Holds the thumb opposed+flexed (flex coupled = opp, like the live driver), then
ramps the index in, watching TWO independent contact signals:
  * fingertip force (reg18 thumb-tip, reg20 index-tip) -- a jump above baseline,
  * index motor-position STALL -- a blocked finger can't reach its commanded reg,
    so (commanded - motor_pos) grows; robust even if the force pads are flaky.

Uses CONTINUOUS writes (single-shot writes don't reliably move the hand). Stop the
teleop service first so the COM port is free. Low speed/force; relaxes on exit.

    conda run -n teleop python tools/calib_pinch.py --port COM7 --slave 1 --opp 670

If no contact is found, the thumb isn't opposed into the index's path -> re-run with
a larger --opp (e.g. 800, 900).
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import hand_driver as hd  # noqa: E402


def hold(driver, regs, dwell, rate=40.0):
    """Continuously command `regs` for `dwell`s so the finger actually moves/settles."""
    period = 1.0 / rate
    end = time.perf_counter() + dwell
    while time.perf_counter() < end:
        driver.write_positions(regs, force=True)
        time.sleep(period)


def read_avg(driver, n=4):
    fb = None
    for _ in range(n):
        f = driver.read_feedback()
        if f is not None:
            fb = f
        time.sleep(0.02)
    return fb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--opp", type=int, default=670, help="thumb opposition; flex coupled = opp")
    ap.add_argument("--finger", type=int, default=2, choices=[2, 3],
                    help="which finger to pinch: 2=index, 3=middle")
    ap.add_argument("--speed", type=int, default=500)
    ap.add_argument("--force", type=int, default=500)
    ap.add_argument("--istart", type=int, default=400)
    ap.add_argument("--istep", type=int, default=50)
    ap.add_argument("--dwell", type=float, default=0.8)
    args = ap.parse_args()

    bus = hd.SerialBus.get(args.port, args.baud)
    if not bus.is_open:
        print("ERROR: cannot open %s -- is the teleop service still running?" % args.port)
        return
    driver = hd.HandDriver("right", args.slave, [], bus, deadband=0)
    driver.write_speed(args.speed)
    driver.write_force(args.force)
    fname = {2: "index", 3: "middle"}[args.finger]

    def pose(fcurl):
        regs = [args.opp, args.opp, 0, 0, 0, 0]
        regs[args.finger] = fcurl
        return regs

    print("opp=%d flex=%d(coupled)  pinch=%s  speed=%d force=%d"
          % (args.opp, args.opp, fname, args.speed, args.force))
    print("WARNING: the thumb opposes and %s curls into it -- keep clear.\n" % fname)

    hold(driver, pose(0), 1.5)  # thumb opposed, finger open -> baseline
    base = read_avg(driver)
    base_tt = base.force_g[0] if base else 0
    base_ft = base.force_g[args.finger] if base else 0
    print("baseline force  thumb_tip=%d  %s_tip=%d\n" % (base_tt, fname, base_ft))

    print("%-7s | %-8s %-8s | %-8s %-8s | %s"
          % (fname, "f_thumb", "f_" + fname, "mot", "cmd-mot", "contact?"))
    print("-" * 64)
    contact = None
    for ix in range(args.istart, 1001, args.istep):
        hold(driver, pose(ix), args.dwell)
        fb = read_avg(driver)
        if fb is None:
            print("%-7d | (no feedback)" % ix)
            continue
        ft, ff = fb.force_g[0], fb.force_g[args.finger]
        mot = fb.motor_pos[args.finger]
        gap = ix - mot
        hit = (ft > base_tt + 150) or (ff > base_ft + 150) or (gap > 60)
        if hit and contact is None:
            contact = ix
        print("%-7d | %-8d %-8d | %-8d %-8d | %s"
              % (ix, ft, ff, mot, gap, "<-- CONTACT" if hit else ""))

    print("\nestimated thumb<->%s contact at reg ~= %s" % (fname, contact))
    if contact is not None:
        print("retargeting emits ~445 at a pinch -> set the finger calib anchor to [445, %d]"
              % contact)
    else:
        print("no contact found -- thumb not opposed into the %s; re-run with larger --opp" % fname)
    hold(driver, [0] * 6, 0.6)
    driver.relax()
    print("relaxed.")


if __name__ == "__main__":
    main()
