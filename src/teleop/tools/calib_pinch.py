"""Find the REAL thumb-index contact point by sweeping flex and watching force.

Drives the hand directly (service must be stopped). Holds the pinch opposition +
index value, ramps thumb flex up while reading fingertip force; a force jump above
baseline = real contact. Then (if needed) ramps the index too. Reveals how much
flex the real hand actually needs to pinch, vs what retargeting commanded.
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from dexhand_teleop import hand_driver as hd  # noqa: E402


def read_force(bus, slave):
    expect = 3 + hd.FEEDBACK_REG_COUNT * 2 + 2
    resp = bus.query(hd.build_read_holding(slave, hd.FEEDBACK_REG_START, hd.FEEDBACK_REG_COUNT), expect)
    if not resp or len(resp) < expect or resp[0] != slave:
        return None
    r = [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(hd.FEEDBACK_REG_COUNT)]
    return r[0], r[1], r[2]  # thumb_tip, thumb_mid, index_tip force (g)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--opp", type=int, default=643)      # thumb opposition (from the live pinch)
    ap.add_argument("--index", type=int, default=445)    # index curl (from the live pinch)
    ap.add_argument("--speed", type=int, default=600)
    ap.add_argument("--force", type=int, default=500)
    args = ap.parse_args()

    bus = hd.SerialBus.get(args.port, args.baud if hasattr(args, "baud") else 115200)
    if not bus.is_open:
        print("ERROR: cannot open", args.port); return
    bus.write(hd.build_write_multiple(args.slave, hd.SPEED_REG_START, [args.speed] * 6))
    bus.write(hd.build_write_multiple(args.slave, hd.FORCE_REG_START, [args.force] * 6))

    def pose(opp, tflex, index):
        bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START,
                                          [opp, tflex, index, 0, 0, 0]))

    print("opp=%d index=%d  (ramping thumb flex; watch the hand)" % (args.opp, args.index))
    pose(args.opp, 300, args.index); time.sleep(1.2)
    base = read_force(bus, args.slave)
    base_tt = base[0] if base else 0
    print("baseline thumb_tip force = %s" % base_tt)
    print("%-11s %-11s %-11s %-11s" % ("thumb_flex", "thumb_tip", "thumb_mid", "index_tip"))
    contact_tf = None
    for tf in range(320, 1001, 60):
        pose(args.opp, tf, args.index); time.sleep(0.45)
        f = read_force(bus, args.slave)
        if f:
            print("%-11d %-11d %-11d %-11d" % (tf, f[0], f[1], f[2]))
            if contact_tf is None and (f[0] > base_tt + 200 or f[1] > base_tt + 200):
                contact_tf = tf
    print("\nthumb-flex contact at reg1 ~= %s (commanded during pinch was 320)" % contact_tf)

    if contact_tf is None:
        print("\nno contact via thumb flex alone; now ramping INDEX at thumb_flex=1000...")
        print("%-11s %-11s %-11s" % ("index", "thumb_tip", "index_tip"))
        for ix in range(args.index, 1001, 60):
            pose(args.opp, 1000, ix); time.sleep(0.45)
            f = read_force(bus, args.slave)
            if f:
                print("%-11d %-11d %-11d" % (ix, f[0], f[2]))
                if f[0] > base_tt + 200 or f[2] > base_tt + 200:
                    print("  -> contact at index=%d, thumb_flex=1000" % ix); break

    time.sleep(0.5)
    bus.write(hd.build_write_multiple(args.slave, hd.CONTROL_REG_START, [0] * 6))
    print("relaxed.")


if __name__ == "__main__":
    main()
