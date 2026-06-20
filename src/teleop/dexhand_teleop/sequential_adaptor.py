"""Underactuated (tendon) sequential coupling for the LZ-SG002 fingers.

The real hand has 6 actuators + 5 passive DOF. Each finger has two segments driven
by ONE tendon: as the motor pulls, the PROXIMAL segment bends first; only after the
proximal reaches its limit A does the DISTAL (fingertip) segment start bending (up to
B). So the relationship between the actuator coordinate (motor travel) theta and the
two physical joints is *sequential*, not the proportional `mimic multiplier=1`:

    q_prox = clip(theta, 0, A)
    q_dist = clip(theta - A, 0, B)

We model theta as the optimized "proximal" target joint, whose URDF limit is extended
to [0, A+B] (= max tendon travel). This adaptor maps theta -> (q_prox, q_dist) for FK,
so dex-retargeting optimizes against the REAL fingertip trajectory, and the register is
just theta / (A+B) * 1000.

Subclasses MimicJointKinematicAdaptor to reuse its index bookkeeping (source = proximal
target joints, mimic = distal joints); only the forward map and its jacobian differ.
"""
import numpy as np
from dex_retargeting.kinematics_adaptor import MimicJointKinematicAdaptor


class SequentialCouplingAdaptor(MimicJointKinematicAdaptor):
    def __init__(self, robot, target_joint_names, source_joint_names,
                 mimic_joint_names, multipliers, offsets):
        super().__init__(robot, target_joint_names, source_joint_names,
                         mimic_joint_names, multipliers, offsets)
        jl = np.asarray(robot.joint_limits)  # (nq, 2): [lower, upper] by dof index
        # A = physical proximal max (source upper was extended to A+B), B = distal max
        self.B = jl[self.idx_pin2mimic, 1]
        self.A = jl[self.idx_pin2source, 1] - self.B
        n = len(self.idx_pin2mimic)
        self._src_scale = np.ones(n)
        self._mim_scale = np.zeros(n)

    def forward_qpos(self, pin_qpos: np.ndarray) -> np.ndarray:
        theta = pin_qpos[self.idx_pin2source].copy()
        pin_qpos[self.idx_pin2source] = np.clip(theta, 0.0, self.A)
        pin_qpos[self.idx_pin2mimic] = np.clip(theta - self.A, 0.0, self.B)
        # gradient routing for this theta: proximal-phase / distal-phase / saturated
        self._src_scale = (theta < self.A).astype(float)
        self._mim_scale = ((theta >= self.A) & (theta < self.A + self.B)).astype(float)
        return pin_qpos

    def backward_jacobian(self, jacobian: np.ndarray) -> np.ndarray:
        target_jacobian = jacobian[..., self.idx_pin2target].copy()
        mimic_cols = jacobian[..., self.idx_pin2mimic]
        for i, src in enumerate(self.idx_target2source):
            target_jacobian[..., src] = (target_jacobian[..., src] * self._src_scale[i]
                                         + mimic_cols[..., i] * self._mim_scale[i])
        return target_jacobian
