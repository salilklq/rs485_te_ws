"""Manual register jogger for the dexterous hand (no glove, no retargeting).

A background thread continuously re-sends the current pose (~30Hz) so the servos
actually move and HOLD it (single-shot writes don't reliably move the hand). Use it
to find the real hand's pinch register values by eye -- exactly how the ground-truth
pinch (thumb_opp=670 flex=679 index=869) was measured on the previous hand.

    conda run -n teleop python tools/jog.py --port COM7 --slave 1

Registers:  0 thumb-opposition, 1 thumb-flex, 2 index, 3 middle, 4 ring, 5 pinky
Commands at the jog> prompt:
    0 670       set reg0 (thumb opp) to 670      (any '<i> <val>', val 0..1000)
    1 500       set reg1 (thumb flex) to 500
    2 850       set reg2 (index) to 850
    f 800       set all four fingers (reg2..5) to 800
    r           read feedback (motor pos / joint angle / fingertip force)
    z           zero all position registers (open hand)
    speed 400   set speed (regs 6..11)      force 300   set force (regs 12..17)
    q           quit (opens hand on exit)

Find a pinch: set thumb opp+flex so the thumb tip aims at a fingertip, then raise
that finger until they just touch; type 'r' and note the reg values.
"""
import argparse
import sys
import threading
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
        print("  (no feedback)"); return
    regs = [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(hd.FEEDBACK_REG_COUNT)]
    print("  motor_pos :", regs[23:29])
    print("  joint_deg :", [round(r * 0.1, 1) for r in regs[13:23]])
    print("  force_g   :", regs[0:6])


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
        print("ERROR: could not open", args.port); return
    print("opened", args.port, "slave", args.slave)
    bus.write(hd.build_write_multiple(args.slave, hd.SPEED_REG_START, [args.speed] * 6))
    bus.write(hd.build_write_multiple(args.slave, hd.FORCE_REG_START, [args.force] * 6))

    state = {"regs": [0, 0, 0, 0, 0, 0], "run": True}

    def resender():
        while state["run"]:
            bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START, state["regs"]))
            time.sleep(0.03)
    threading.Thread(target=resender, daemon=True).start()

    def show():
        print("  ->", dict(zip(NAMES, state["regs"])))

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
                for k in range(6):
                    state["regs"][k] = 0
                show()
            elif c == "r":
                read_fb(bus, args.slave)
            elif c == "f" and len(parts) == 2:
                v = max(0, min(1000, int(parts[1])))
                for k in (2, 3, 4, 5):
                    state["regs"][k] = v
                show()
            elif c == "speed" and len(parts) == 2:
                bus.write(hd.build_write_multiple(args.slave, hd.SPEED_REG_START, [int(parts[1])] * 6))
                print("  speed set", parts[1])
            elif c == "force" and len(parts) == 2:
                bus.write(hd.build_write_multiple(args.slave, hd.FORCE_REG_START, [int(parts[1])] * 6))
                print("  force set", parts[1])
            elif c.isdigit() and len(parts) == 2:
                i = int(c)
                if 0 <= i <= 5:
                    state["regs"][i] = max(0, min(1000, int(parts[1]))); show()
                else:
                    print("  reg index must be 0..5")
            else:
                print("  ? see commands above")
        except (ValueError, IndexError):
            print("  bad input")

    state["run"] = False
    time.sleep(0.1)
    bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START, [0] * 6))
    print("opened hand. bye.")


if __name__ == "__main__":
    main()
