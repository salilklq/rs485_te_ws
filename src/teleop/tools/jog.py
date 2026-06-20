"""Manual register jogger for the dexterous hand (no glove, no retargeting).

Directly writes position registers 0..5 over RS485 so you can characterize the
real hand's motion vs commanded value -- essential for thumb calibration.

    conda run -n teleop python tools/jog.py --port COM5 --slave 1

Registers:  0 thumb-rotation/opposition, 1 thumb-flex, 2 index, 3 middle, 4 ring, 5 pinky
Commands at the jog> prompt:
    0 1000      set reg0 to 1000          (any '<i> <val>', val 0..1000)
    1 500       set reg1 to 500
    f 800       set all four fingers (reg2..5) to 800
    sweep 1     sweep reg1 0->1000 in steps, printing motor feedback at each
    r           read feedback (motor pos / joint angle / fingertip force)
    z           zero all position registers (relax)
    speed 300   set speed (regs 6..11)     force 200  set force (regs 12..17)
    q           quit (zeros on exit)
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import hand_driver as hd  # noqa: E402

NAMES = ["thumb_opp", "thumb_flex", "index", "middle", "ring", "pinky"]


def read_fb(bus, slave):
    expect = 3 + hd.FEEDBACK_REG_COUNT * 2 + 2
    resp = bus.query(hd.build_read_holding(slave, hd.FEEDBACK_REG_START, hd.FEEDBACK_REG_COUNT), expect)
    if not resp or len(resp) < expect or resp[0] != slave or resp[1] != 0x03:
        print("  (no feedback)")
        return
    regs = [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(hd.FEEDBACK_REG_COUNT)]
    motor = regs[23:29]
    angle = [r * 0.1 for r in regs[13:23]]
    force = regs[0:13]
    print("  motor_pos :", motor)
    print("  joint_deg :", [round(a, 1) for a in angle])
    print("  force_g   : thumb_tip=%d index_tip=%d middle_tip=%d" % (force[0], force[2], force[4]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=400)
    ap.add_argument("--force", type=int, default=300)
    args = ap.parse_args()

    bus = hd.SerialBus.get(args.port, args.baud)
    if not bus.is_open:
        print("ERROR: could not open", args.port)
        return
    print("opened", args.port, "slave", args.slave)
    bus.write(hd.build_write_multiple(args.slave, hd.SPEED_REG_START, [args.speed] * 6))
    bus.write(hd.build_write_multiple(args.slave, hd.FORCE_REG_START, [args.force] * 6))
    regs = [0] * 6

    def send():
        bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START, regs))
        print("  ->", dict(zip(NAMES, regs)))

    print(__doc__)
    while True:
        try:
            parts = input("jog> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not parts:
            continue
        c = parts[0].lower()
        try:
            if c == "q":
                break
            elif c == "z":
                regs = [0] * 6; send()
            elif c == "r":
                read_fb(bus, args.slave)
            elif c == "f" and len(parts) == 2:
                v = max(0, min(1000, int(parts[1])))
                regs[2] = regs[3] = regs[4] = regs[5] = v; send()
            elif c == "speed" and len(parts) == 2:
                bus.write(hd.build_write_multiple(args.slave, hd.SPEED_REG_START, [int(parts[1])] * 6))
                print("  speed set", parts[1])
            elif c == "force" and len(parts) == 2:
                bus.write(hd.build_write_multiple(args.slave, hd.FORCE_REG_START, [int(parts[1])] * 6))
                print("  force set", parts[1])
            elif c == "sweep" and len(parts) == 2:
                i = int(parts[1])
                for v in range(0, 1001, 200):
                    regs[i] = v; send(); time.sleep(0.6); read_fb(bus, args.slave)
            elif c.isdigit() and len(parts) == 2:
                i = int(c); regs[i] = max(0, min(1000, int(parts[1]))); send()
            else:
                print("  ? see commands above")
        except (ValueError, IndexError):
            print("  bad input")

    regs = [0] * 6
    bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START, regs))
    print("zeroed. bye.")


if __name__ == "__main__":
    main()
