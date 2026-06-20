"""Thin wrapper over dex-retargeting SeqRetargeting for one hand.

Loads a DexPilot config, and per frame turns 21 MANO-frame keypoints into a
{joint_name: angle} dict. scaling_factor and low_pass_alpha are live-tunable.
"""
from pathlib import Path
from typing import Dict, List

import numpy as np

from dex_retargeting import yourdfpy as _urdf
from dex_retargeting.retargeting_config import RetargetingConfig, parse_mimic_joint

from .sequential_adaptor import SequentialCouplingAdaptor


class HandRetargeter:
    def __init__(self, config_path: str, urdf_dir: str, coupling: str = "sequential"):
        RetargetingConfig.set_default_urdf_dir(str(urdf_dir))
        self.config = RetargetingConfig.load_from_file(config_path)
        self.retargeting = self.config.build()
        self.optimizer = self.retargeting.optimizer

        # Replace dex-retargeting's linear mimic adaptor with the real hand's
        # sequential (underactuated tendon) coupling: proximal bends first, then distal.
        if coupling == "sequential":
            robot_urdf = _urdf.URDF.load(self.config.urdf_path, build_scene_graph=False)
            has_mimic, src, mim, mult, off = parse_mimic_joint(robot_urdf)
            if has_mimic:
                self.optimizer.set_kinematic_adaptor(SequentialCouplingAdaptor(
                    self.optimizer.robot, self.optimizer.target_joint_names, src, mim, mult, off))

        # Human keypoint correspondence (DexPilot: (2, N) origin/task indices).
        self._indices = self.optimizer.target_link_human_indices

        # Joint ordering of the value returned by retarget()
        self.joint_names: List[str] = list(self.retargeting.joint_names)

        # Optimized (target) joints in config/register order + their limits.
        self.target_joint_names: List[str] = list(self.optimizer.target_joint_names)
        # optimizer joint_limits would include mimic; pull per-target limits via SeqRetargeting
        self.target_limits = np.asarray(self.retargeting.joint_limits, dtype=float)  # (6,2)

    # -- live tuning -----------------------------------------------------------
    @property
    def scaling(self) -> float:
        return float(self.optimizer.scaling)

    @scaling.setter
    def scaling(self, value: float):
        self.optimizer.scaling = float(value)

    @property
    def low_pass_alpha(self) -> float:
        return float(self.retargeting.filter.alpha) if self.retargeting.filter else 1.0

    @low_pass_alpha.setter
    def low_pass_alpha(self, value: float):
        if self.retargeting.filter is not None:
            self.retargeting.filter.alpha = float(np.clip(value, 0.0, 1.0))

    def reset(self):
        self.retargeting.reset()
        if self.retargeting.filter is not None:
            self.retargeting.filter.reset()

    # -- per-frame -------------------------------------------------------------
    def retarget(self, joint_pos_mano: np.ndarray) -> Dict[str, float]:
        """joint_pos_mano: (21,3) in MANO frame -> {joint_name: angle_rad}."""
        origin = self._indices[0, :]
        task = self._indices[1, :]
        ref_value = joint_pos_mano[task, :] - joint_pos_mano[origin, :]
        qpos = self.retargeting.retarget(ref_value)
        return dict(zip(self.joint_names, qpos))

    def target_qpos(self, qpos_by_name: Dict[str, float] = None) -> np.ndarray:
        """The 6 actuator coordinates (register order) -> drives reg0..5.

        Returns the optimizer's raw target values (theta), i.e. motor travel before
        the sequential clamp — this is what maps linearly to the 0..1000 register.
        (For proportional coupling it equals the proximal joint angle.)
        """
        return np.asarray(self.retargeting.last_qpos, dtype=float)

    def expand_targets(self, target_angles: np.ndarray) -> np.ndarray:
        """6 target joint angles (register order) -> full pinocchio qpos (mimic filled).

        Used to drive the 3D viewer from the hand's motor-position feedback.
        """
        q = np.zeros(len(self.joint_names), dtype=float)
        q[self.optimizer.idx_pin2target] = np.asarray(target_angles, dtype=float)
        if self.optimizer.adaptor is not None:
            q = self.optimizer.adaptor.forward_qpos(q)
        return q
