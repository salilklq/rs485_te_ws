"""Real-time 3D visualization of the retargeted hand(s) on the real URDF (meshcat).

Per hand we show two overlaid models:
  * commanded  (solid, light blue)  — driven by the retargeted qpos
  * actual     (translucent ghost)  — driven by the hand's motor-position feedback
so you can both verify the mapping (commanded follows your hand) and monitor the
real hand (ghost vs commanded). Left/right hands are offset along Y.

Browser-based (WebGL); meshcat bundles its own front-end assets — works offline.
"""
import os
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

try:
    import meshcat.transformations as mtf
except Exception:  # pragma: no cover
    mtf = None

_HAND_OFFSET = {"right": [0.0, -0.12, 0.0], "left": [0.0, 0.12, 0.0]}
_CMD_COLOR = [0.62, 0.74, 0.95, 1.0]
_ACTUAL_COLOR = [0.95, 0.55, 0.22, 0.35]


class _HandVisual:
    def __init__(self, name, urdf_path, assets_dir, shared_viewer):
        self.name = name
        robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=[assets_dir])
        self.model = robot.model
        self.q0 = pin.neutral(self.model)

        # commanded model (shares the meshcat server; first hand creates it)
        self.cmd = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        self.cmd.initViewer(viewer=shared_viewer, open=False)
        self.cmd.loadViewerModel(rootNodeName=f"{name}_cmd", visual_color=_CMD_COLOR)
        self.viewer = self.cmd.viewer

        # actual (ghost) model — always shares the same server
        robot2 = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=[assets_dir])
        self.actual = MeshcatVisualizer(robot2.model, robot2.collision_model, robot2.visual_model)
        self.actual.initViewer(viewer=self.viewer, open=False)
        self.actual.loadViewerModel(rootNodeName=f"{name}_actual", visual_color=_ACTUAL_COLOR)

        if mtf is not None:
            T = mtf.translation_matrix(_HAND_OFFSET.get(name, [0, 0, 0]))
            self.viewer[f"{name}_cmd"].set_transform(T)
            self.viewer[f"{name}_actual"].set_transform(T)

        self._actual_visible = True
        self.set_actual_visible(False)
        self.cmd.display(self.q0)

    def set_actual_visible(self, visible: bool):
        if visible != self._actual_visible:
            self.viewer[f"{self.name}_actual"].set_property("visible", bool(visible))
            self._actual_visible = visible

    def update(self, q_cmd, q_actual):
        if q_cmd is not None and len(q_cmd) == self.model.nq:
            self.cmd.display(np.asarray(q_cmd))
        if q_actual is not None and len(q_actual) == self.model.nq:
            self.set_actual_visible(True)
            self.actual.display(np.asarray(q_actual))
        else:
            self.set_actual_visible(False)


class HandSceneViz:
    """Manage one meshcat server showing all enabled hands."""

    def __init__(self, hands):
        """hands: dict name -> (urdf_path, assets_dir)."""
        self._hands = {}
        shared = None
        for name, (urdf_path, assets_dir) in hands.items():
            hv = _HandVisual(name, urdf_path, assets_dir, shared)
            shared = hv.viewer
            self._hands[name] = hv
        self.viewer = shared

    @property
    def url(self) -> str:
        try:
            return self.viewer.url()
        except Exception:
            return "http://127.0.0.1:7000/static/"

    def update(self, name, q_cmd, q_actual=None):
        hv = self._hands.get(name)
        if hv is not None:
            hv.update(q_cmd, q_actual)

    def close(self):
        """Kill the meshcat server subprocess (it outlives this process otherwise)."""
        try:
            proc = getattr(getattr(self.viewer, "window", None), "server_proc", None)
            if proc is not None:
                proc.kill()
        except Exception:
            pass
