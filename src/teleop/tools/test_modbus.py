"""Offline checks for the Modbus-RTU framing/CRC and feedback parsing (no hardware)."""
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from dexhand_teleop import hand_driver as hd  # noqa: E402


def main():
    ok = True

    # Known Modbus-RTU CRC16 vectors
    c1 = hd.modbus_crc16(bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01]))
    print("CRC(01 03 00 00 00 01) =", hex(c1), "expect 0x0a84")
    ok &= (c1 == 0x0A84)
    c2 = hd.modbus_crc16(bytes([0x01, 0x04, 0x02, 0xFF, 0xFF]))
    # 16-bit value 0x80B8; on the wire (low byte first) it is B8 80
    print("CRC(01 04 02 FF FF)    =", hex(c2), "-> wire B8 80")
    ok &= (c2 == 0x80B8)

    # Write-multiple frame for positions 0..5
    f = hd.build_write_multiple(1, 0, [0, 500, 500, 500, 500, 500])
    print("write regs0-5:", f.hex(" "))
    ok &= (hd.modbus_crc16(f[:-2]) == (f[-2] | (f[-1] << 8)))
    ok &= (f[0] == 1 and f[1] == 0x10 and f[5] == 6 and f[6] == 12)

    # qpos -> registers mapping
    joints = hd.joints_from_config(
        [{"name": "j%d" % i, "qmin": 0.0, "qmax": 1.5708, "out_lo": 0, "out_hi": 1000}
         for i in range(6)])
    drv = hd.HandDriver("right", 1, joints, hd.SerialBus.get("", 115200))
    regs = drv.qpos_to_registers([0.0, 0.7854, 1.5708, 1.5708, 0.0, 0.0])
    print("qpos->regs:", regs, "expect ~[0,500,1000,1000,0,0]")
    ok &= (regs[0] == 0 and abs(regs[1] - 500) <= 1 and regs[2] == 1000)

    # Feedback parse round-trip (29 regs)
    regs_in = list(range(900, 900 + 29))
    body = bytes([1, 3, 58]) + b"".join(struct.pack(">H", r) for r in regs_in)
    crc = hd.modbus_crc16(body)
    resp = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    class FakeBus:
        is_open = True
        def query(self, frame, n):
            return resp

    drv2 = hd.HandDriver("right", 1, joints, FakeBus())
    fb = drv2.read_feedback()
    print("feedback: force0=%d angle0=%.1f motor=%s" %
          (fb.force_g[0], fb.joint_angle_deg[0], fb.motor_pos))
    ok &= (fb.force_g[0] == 900 and abs(fb.joint_angle_deg[0] - 91.3) < 0.01
           and fb.motor_pos == regs_in[23:29])

    print("\nRESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
