"""Hold one fixed register pose on the hand (continuous writes) for inspection.

    python tools/hold_pose.py --port COM7 --regs 1000,500,850,0,0,0 --secs 5 [--keep]

regs = opp,flex,index,middle,ring,pinky (0..1000). Continuous writes (single-shot
don't reliably move the hand). Prints motor_pos + fingertip force. --keep leaves the
hand in the pose (no relax) so you can look at it; otherwise it relaxes on exit.
Stop the teleop service first so the COM port is free.
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import hand_driver as hd  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--regs", required=True, help="6 comma values opp,flex,index,middle,ring,pinky")
    ap.add_argument("--speed", type=int, default=500)
    ap.add_argument("--force", type=int, default=500)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--keep", action="store_true", help="leave hand in pose (no relax on exit)")
    args = ap.parse_args()

    regs = [int(x) for x in args.regs.split(",")]
    if len(regs) != 6:
        print("ERROR: need 6 comma-separated regs"); return
    bus = hd.SerialBus.get(args.port, args.baud)
    if not bus.is_open:
        print("ERROR: cannot open %s -- is the teleop service still running?" % args.port); return
    d = hd.HandDriver("right", args.slave, [], bus, deadband=0)
    d.write_speed(args.speed)
    d.write_force(args.force)

    end = time.perf_counter() + args.secs
    while time.perf_counter() < end:
        d.write_positions(regs, force=True)
        time.sleep(0.025)
    fb = d.read_feedback()
    print("held regs        :", regs)
    if fb:
        print("motor_pos        :", fb.motor_pos)
        print("force_g[0:6]     :", fb.force_g[0:6])
    if args.keep:
        print("left in pose (run with --regs 0,0,0,0,0,0 to open).")
    else:
        d.relax()
        print("relaxed.")


if __name__ == "__main__":
    main()
