"""dexhand_teleop: MANUS glove -> dex-retargeting -> RS485 dexterous hand teleop.

Pipeline (hybrid, see teleop/README.md):
    C++ ManusKeypointStreamer --UDP--> manus_receiver
        -> retarget (dex-retargeting DexPilot) -> qpos
        -> hand_driver (qpos -> reg0..5 -> Modbus-RTU RS485)
        + panel (live tuning / calibration web UI)
"""
__all__ = [
    "protocol",
    "keypoints",
    "manus_receiver",
    "retarget",
    "hand_driver",
]
