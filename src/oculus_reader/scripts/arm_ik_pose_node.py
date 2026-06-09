#!/usr/bin/env python3
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import casadi
except ImportError:
    casadi = None

try:
    from pinocchio import casadi as cpin
except ImportError:
    cpin = None


R_MAP_RIGHT = np.array(
    [
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)
R_MAP_LEFT = np.array(
    [
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=float,
)

def xyzrpy_to_mat(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = rotation_to_matrix(Rotation.from_euler("xyz", [roll, pitch, yaw]))
    mat[:3, 3] = np.array([x, y, z])
    return mat


def rotation_to_matrix(rotation: Rotation) -> np.ndarray:
    if hasattr(rotation, "as_matrix"):
        return rotation.as_matrix()
    return rotation.as_dcm()


def rotation_from_matrix(matrix: np.ndarray) -> Rotation:
    if hasattr(Rotation, "from_matrix"):
        return Rotation.from_matrix(matrix)
    return Rotation.from_dcm(matrix)


def normalize_quat_xyzw(quat) -> np.ndarray:
    arr = np.asarray(quat, dtype=float).reshape(-1)
    if arr.shape != (4,) or not np.all(np.isfinite(arr)):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return arr / norm


def pose_to_mat(pose: Pose) -> np.ndarray:
    mat = np.eye(4)
    quat = normalize_quat_xyzw([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    mat[:3, :3] = rotation_to_matrix(Rotation.from_quat(quat))
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def pose_to_position_rotation(pose: Pose) -> Tuple[np.ndarray, Rotation]:
    position = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    quat = normalize_quat_xyzw([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    if not np.all(np.isfinite(position)):
        raise ValueError("Pose position contains non-finite values")
    return position, Rotation.from_quat(quat)


def position_rotation_to_mat(position: np.ndarray, rotation: Rotation) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = rotation_to_matrix(rotation)
    mat[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return mat


def make_operational_frame(
    model,
    frame_name: str,
    parent_joint_name: str,
    placement,
):
    parent_joint_id = model.getJointId(parent_joint_name)
    try:
        return pin.Frame(frame_name, parent_joint_id, placement, pin.FrameType.OP_FRAME)
    except Exception:
        parent_frame_id = model.getFrameId(parent_joint_name)
        if parent_frame_id >= len(model.frames):
            parent_frame_id = 0
        return pin.Frame(frame_name, parent_joint_id, parent_frame_id, placement, pin.FrameType.OP_FRAME)


class ArmIK:
    def __init__(
        self,
        urdf_path: str,
        package_dirs: List[str],
        locked_joints: List[str],
        ee_parent_joint: str,
        ee_frame_name: str,
        tool_pre_rot_rpy: List[float],
        tool_translation_xyz: List[float],
        collision_pairs_flat: List[int],
        w_pos: float,
        w_ori: float,
        w_reg: float,
        w_smooth: float,
        ipopt_max_iter: int,
        ipopt_tol: float,
        enable_visualization: bool,
        viewer_open_browser: bool,
        viewer_model_name: str,
        viewer_target_frame_name: str,
        viewer_axis_length: float,
        viewer_axis_width: float,
    ):
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)
        unique_locked_joints = self._deduplicate_locked_joints(locked_joints)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=unique_locked_joints,
            reference_configuration=np.zeros(self.robot.model.nq),
        )

        first = xyzrpy_to_mat(0.0, 0.0, 0.0, tool_pre_rot_rpy[0], tool_pre_rot_rpy[1], tool_pre_rot_rpy[2])
        second = xyzrpy_to_mat(tool_translation_xyz[0], tool_translation_xyz[1], tool_translation_xyz[2], 0.0, 0.0, 0.0)
        ee_mat = first @ second
        quat = rotation_from_matrix(ee_mat[:3, :3]).as_quat()  # x y z w

        self.reduced_robot.model.addFrame(
            make_operational_frame(
                self.reduced_robot.model,
                ee_frame_name,
                ee_parent_joint,
                pin.SE3(
                    pin.Quaternion(quat[3], quat[0], quat[1], quat[2]),
                    np.array(ee_mat[:3, 3]),
                ),
            )
        )
        # addFrame() changes the model frame count after buildReducedRobot() has
        # already created data. Recreate data so data.oMf includes the new EE
        # frame; otherwise FK/IK frame access can raise "Index out of range".
        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.geom_model = self.reduced_robot.collision_model
        for i in range(0, len(collision_pairs_flat), 2):
            a = collision_pairs_flat[i]
            b = collision_pairs_flat[i + 1]
            self.geom_model.addCollisionPair(pin.CollisionPair(a, b))
        self.geometry_data = pin.GeometryData(self.geom_model)

        self.ee_id = self.reduced_robot.model.getFrameId(ee_frame_name)
        self.w_pos = w_pos
        self.w_ori = w_ori
        self.w_reg = w_reg
        self.w_smooth = w_smooth
        self.ipopt_max_iter = ipopt_max_iter
        self.ipopt_tol = ipopt_tol
        self.backend_name = "scipy_least_squares"

        self.cmodel = None
        self.cdata = None
        self.cq = None
        self.ctf = None
        self.error = None
        self.opti = None
        self.var_q = None
        self.param_q_prev = None
        self.param_tf = None
        if casadi is not None and cpin is not None:
            self.backend_name = "casadi_ipopt"
            self.cmodel = cpin.Model(self.reduced_robot.model)
            self.cdata = self.cmodel.createData()
            self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
            self.ctf = casadi.SX.sym("tf", 4, 4)
            cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

            self.error = casadi.Function(
                "error",
                [self.cq, self.ctf],
                [
                    casadi.vertcat(
                        cpin.log6(self.cdata.oMf[self.ee_id].inverse() * cpin.SE3(self.ctf)).vector
                    )
                ],
            )

            self.opti = casadi.Opti()
            self.var_q = self.opti.variable(self.reduced_robot.model.nq)
            self.param_q_prev = self.opti.parameter(self.reduced_robot.model.nq)
            self.param_tf = self.opti.parameter(4, 4)

            error_vec = self.error(self.var_q, self.param_tf)
            pos_error = error_vec[:3]
            ori_error = error_vec[3:]
            total_cost = casadi.sumsqr(w_pos * pos_error) + casadi.sumsqr(w_ori * ori_error)
            regularization = casadi.sumsqr(self.var_q)
            smooth_cost = casadi.sumsqr(self.var_q - self.param_q_prev)
            self.opti.minimize(total_cost + w_reg * regularization + w_smooth * smooth_cost)

            self.opti.subject_to(
                self.opti.bounded(
                    self.reduced_robot.model.lowerPositionLimit,
                    self.var_q,
                    self.reduced_robot.model.upperPositionLimit,
                )
            )

            self.opti.solver(
                "ipopt",
                {"ipopt": {"print_level": 0, "max_iter": ipopt_max_iter, "tol": ipopt_tol}, "print_time": False},
            )

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)
        self.enable_visualization = enable_visualization
        self.vis = None
        self.viewer_target_frame_name = viewer_target_frame_name
        if self.enable_visualization:
            self._init_visualizer(
                open_browser=viewer_open_browser,
                viewer_model_name=viewer_model_name,
                target_frame_name=viewer_target_frame_name,
                axis_length=viewer_axis_length,
                axis_width=viewer_axis_width,
            )

    def _deduplicate_locked_joints(self, locked_joints: List[str]) -> List[str]:
        unique_names: List[str] = []
        seen_joint_ids = set()
        for joint_name in locked_joints:
            try:
                joint_id = self.robot.model.getJointId(joint_name)
            except Exception:
                continue
            if joint_id <= 0:
                continue
            if joint_id in seen_joint_ids:
                continue
            seen_joint_ids.add(joint_id)
            unique_names.append(joint_name)
        return unique_names

    @property
    def nq(self) -> int:
        return self.reduced_robot.model.nq

    def active_joint_names(self) -> List[str]:
        names = [n for n in self.reduced_robot.model.names if n != "universe"]
        return names

    def sync_state(self, q_current: List[float]) -> None:
        q = np.array(q_current, dtype=float)
        if q.shape[0] == self.nq:
            self.init_data = q
            self.history_data = q

    def end_effector_pose(self, q_current: Optional[np.ndarray] = None) -> np.ndarray:
        q = self.history_data if q_current is None else np.asarray(q_current, dtype=float)
        if q.shape[0] != self.nq or not np.all(np.isfinite(q)):
            raise ValueError(f"Expected finite q with length {self.nq}, got shape {q.shape}")

        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)
        if self.ee_id >= len(self.reduced_robot.data.oMf):
            raise RuntimeError(
                f"EE frame id {self.ee_id} is outside data.oMf length "
                f"{len(self.reduced_robot.data.oMf)}"
            )
        placement = self.reduced_robot.data.oMf[self.ee_id]
        mat = np.eye(4)
        mat[:3, :3] = np.asarray(placement.rotation, dtype=float)
        mat[:3, 3] = np.asarray(placement.translation, dtype=float).reshape(3)
        return mat

    def pose_error_norms(self, q: np.ndarray, target_pose: np.ndarray) -> Tuple[float, float]:
        err = self._pose_error(np.asarray(q, dtype=float), target_pose)
        return float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

    def solve(self, target_pose: np.ndarray) -> np.ndarray:
        self.display_target(target_pose)

        if self.opti is not None:
            sol_q = self._solve_with_casadi(target_pose)
        else:
            sol_q = self._solve_with_scipy(target_pose)

        sol_q = np.asarray(sol_q, dtype=float).reshape(-1)
        if sol_q.shape[0] != self.nq or not np.all(np.isfinite(sol_q)):
            raise RuntimeError("IK solver returned an invalid joint vector")

        self.init_data = sol_q
        self.history_data = sol_q
        self.display_solution(sol_q)
        return sol_q

    def _solve_with_casadi(self, target_pose: np.ndarray) -> np.ndarray:
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_tf, target_pose)
        self.opti.solve_limited()
        return np.array(self.opti.value(self.var_q)).reshape(-1)

    def _pose_error(self, q: np.ndarray, target_pose: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        pin.updateFramePlacements(self.reduced_robot.model, self.reduced_robot.data)
        if self.ee_id >= len(self.reduced_robot.data.oMf):
            raise RuntimeError(
                f"EE frame id {self.ee_id} is outside data.oMf length "
                f"{len(self.reduced_robot.data.oMf)}"
            )
        target_se3 = pin.SE3(
            np.asarray(target_pose[:3, :3], dtype=float),
            np.asarray(target_pose[:3, 3], dtype=float),
        )
        error = pin.log6(self.reduced_robot.data.oMf[self.ee_id].inverse() * target_se3).vector
        return np.asarray(error, dtype=float).reshape(-1)

    def _solve_with_scipy(self, target_pose: np.ndarray) -> np.ndarray:
        lower = np.asarray(self.reduced_robot.model.lowerPositionLimit, dtype=float)
        upper = np.asarray(self.reduced_robot.model.upperPositionLimit, dtype=float)
        seed = np.clip(self.init_data, lower, upper)
        prev = np.clip(self.history_data, lower, upper)

        sqrt_reg = math.sqrt(max(self.w_reg, 0.0))
        sqrt_smooth = math.sqrt(max(self.w_smooth, 0.0))

        def residual(q: np.ndarray) -> np.ndarray:
            err = self._pose_error(q, target_pose)
            return np.concatenate(
                [
                    self.w_pos * err[:3],
                    self.w_ori * err[3:],
                    sqrt_reg * q,
                    sqrt_smooth * (q - prev),
                ]
            )

        result = least_squares(
            residual,
            seed,
            bounds=(lower, upper),
            max_nfev=max(1, self.ipopt_max_iter),
            ftol=self.ipopt_tol,
            xtol=self.ipopt_tol,
            gtol=self.ipopt_tol,
        )
        return np.clip(np.asarray(result.x, dtype=float).reshape(-1), lower, upper)

    def check_self_collision(self, q: np.ndarray) -> bool:
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        pin.updateGeometryPlacements(self.reduced_robot.model, self.reduced_robot.data, self.geom_model, self.geometry_data)
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)

    def _init_visualizer(
        self,
        open_browser: bool,
        viewer_model_name: str,
        target_frame_name: str,
        axis_length: float,
        axis_width: float,
    ) -> None:
        import meshcat.geometry as mg
        from pinocchio.visualize import MeshcatVisualizer

        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=open_browser)
        self.vis.loadViewerModel(viewer_model_name)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        frame_axis_positions = (
            np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        frame_axis_colors = (
            np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        self.vis.viewer[target_frame_name].set_object(
            mg.LineSegments(
                mg.PointsGeometry(position=axis_length * frame_axis_positions, color=frame_axis_colors),
                mg.LineBasicMaterial(linewidth=axis_width, vertexColors=True),
            )
        )

    def display_target(self, target_pose: np.ndarray) -> None:
        if self.vis is not None:
            self.vis.viewer[self.viewer_target_frame_name].set_transform(target_pose)

    def display_solution(self, q: np.ndarray) -> None:
        if self.vis is not None:
            self.vis.display(q)


class ArmIKPoseNode(Node):
    def __init__(self):
        super().__init__("arm_ik_pose_node")

        self.declare_parameter("robot_description_package", "nero_description")
        self.declare_parameter("urdf_relative_path", "urdf/nero.urdf")
        self.declare_parameter("locked_joints", ["joint8"])
        self.declare_parameter("ee_parent_joint", "joint7")
        self.declare_parameter("ee_frame_name", "ee")
        self.declare_parameter("tool_pre_rot_rpy", [-1.57, 0.0, -1.57])
        self.declare_parameter("tool_translation_xyz", [0.0, 0.023, 0.064])
        self.declare_parameter("collision_pairs_flat", [5, 0, 5, 1, 5, 2, 5, 3])
        self.declare_parameter("enable_collision_check", False)

        self.declare_parameter("w_pos", 20.0)
        self.declare_parameter("w_ori", 2.0)
        self.declare_parameter("w_reg", 0.01)
        self.declare_parameter("w_smooth", 2.0)
        self.declare_parameter("ipopt_max_iter", 50)
        self.declare_parameter("ipopt_tol", 1e-4)
        self.declare_parameter("enable_visualization", False)
        self.declare_parameter("viewer_open_browser", True)
        self.declare_parameter("viewer_model_name", "pinocchio")
        self.declare_parameter("viewer_target_frame_name", "ee_target")
        self.declare_parameter("viewer_axis_length", 0.1)
        self.declare_parameter("viewer_axis_width", 10.0)

        self.declare_parameter("pose_stamped_topic", "")
        self.declare_parameter("feedback_joint_topic", "")
        self.declare_parameter("pin_joint_status_topic", "pin_joint_status")
        # NOTE: Empty list default is inferred as BYTE_ARRAY in rclpy.
        # Use string array default to keep YAML STRING_ARRAY override compatible.
        self.declare_parameter("output_joint_names", [""])
        self.declare_parameter("delta_mapping_enabled", True)
        self.declare_parameter("hand_name", "auto")
        self.declare_parameter("control_enable_topic", "")
        self.declare_parameter("workspace_radius", 0.8)
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("input_timeout_sec", 0.5)
        self.declare_parameter("require_feedback_before_latch", True)
        self.declare_parameter("max_solution_pos_error", 0.05)
        self.declare_parameter("max_solution_ori_error", 0.5)
        self.declare_parameter("orientation_mode", "track_full")

        # ── YAML-configurable position mapping matrices (flat 9-element, row-major) ──
        self.declare_parameter("position_map_left", [0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
        self.declare_parameter("position_map_right", [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0])

        # ── Asymmetric workspace clamp (relative to latch point) ────
        self.declare_parameter("max_delta_x_forward", 0.45)
        self.declare_parameter("max_delta_x_backward", 0.25)
        self.declare_parameter("max_delta_y_positive", 0.30)
        self.declare_parameter("max_delta_y_negative", 0.30)
        self.declare_parameter("max_delta_z_positive", 0.35)
        self.declare_parameter("max_delta_z_negative", 0.35)

        # ── Position target filtering ───────────────────────────────
        self.declare_parameter("target_pos_alpha", 0.55)
        self.declare_parameter("max_step_pos", 0.015)
        self.declare_parameter("deadband_pos", 0.0015)

        # ── Adaptive tremor filter ──────────────────────────────────
        self.declare_parameter("enable_adaptive_tremor_filter", True)
        self.declare_parameter("still_motion_threshold_pos", 0.008)
        self.declare_parameter("still_motion_threshold_rot", 0.06)
        self.declare_parameter("still_filter_alpha_scale", 0.60)
        self.declare_parameter("still_deadband_pos_scale", 1.50)
        self.declare_parameter("still_rot_deadband_scale", 1.50)

        # ── Orientation tracking parameters ─────────────────────────
        self.declare_parameter("orientation_gain", 0.35)
        self.declare_parameter("max_orientation_delta_rad", 0.60)
        self.declare_parameter("max_step_angle", 0.06)
        self.declare_parameter("target_rot_alpha", 0.35)
        self.declare_parameter("rot_deadband", 0.015)
        self.declare_parameter("orientation_axis_gain", [1.0, 1.0, 1.0])
        self.declare_parameter("orientation_map_left", [0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0])
        self.declare_parameter("orientation_map_right", [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0])

        package_name = self.get_parameter("robot_description_package").value
        urdf_rel = self.get_parameter("urdf_relative_path").value
        locked_joints = list(self.get_parameter("locked_joints").value)
        ee_parent_joint = self.get_parameter("ee_parent_joint").value
        ee_frame_name = self.get_parameter("ee_frame_name").value
        tool_pre_rot_rpy = list(self.get_parameter("tool_pre_rot_rpy").value)
        tool_translation_xyz = list(self.get_parameter("tool_translation_xyz").value)
        collision_pairs_flat = [int(v) for v in self.get_parameter("collision_pairs_flat").value]
        if len(collision_pairs_flat) % 2 != 0:
            raise ValueError("collision_pairs_flat length must be even, e.g. [5,0,5,1].")
        enable_collision_check = bool(self.get_parameter("enable_collision_check").value)

        w_pos = float(self.get_parameter("w_pos").value)
        w_ori = float(self.get_parameter("w_ori").value)
        w_reg = float(self.get_parameter("w_reg").value)
        w_smooth = float(self.get_parameter("w_smooth").value)
        ipopt_max_iter = int(self.get_parameter("ipopt_max_iter").value)
        ipopt_tol = float(self.get_parameter("ipopt_tol").value)
        enable_visualization = bool(self.get_parameter("enable_visualization").value)
        viewer_open_browser = bool(self.get_parameter("viewer_open_browser").value)
        viewer_model_name = self.get_parameter("viewer_model_name").value
        viewer_target_frame_name = self.get_parameter("viewer_target_frame_name").value
        viewer_axis_length = float(self.get_parameter("viewer_axis_length").value)
        viewer_axis_width = float(self.get_parameter("viewer_axis_width").value)

        pose_stamped_topic = str(self.get_parameter("pose_stamped_topic").value).strip()
        feedback_joint_topic = str(self.get_parameter("feedback_joint_topic").value).strip()
        pin_joint_status_topic = str(self.get_parameter("pin_joint_status_topic").value).strip()
        self.delta_mapping_enabled = bool(self.get_parameter("delta_mapping_enabled").value)
        self.hand_name = self._resolve_hand_name(
            str(self.get_parameter("hand_name").value),
            pose_stamped_topic,
            pin_joint_status_topic,
        )
        self.r_map = R_MAP_LEFT.copy() if self.hand_name == "left" else R_MAP_RIGHT.copy()
        # ── Side-mount base rotation matrices (Rotation objects) ─────
        # Right arm: robot +X forward, +Y up,    +Z right
        self.R_map_right = rotation_from_matrix(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]))
        # Left  arm: robot +X forward, +Y down,  +Z left
        self.R_map_left  = rotation_from_matrix(np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]]))
        self.control_enable_topic = str(self.get_parameter("control_enable_topic").value).strip()
        self.workspace_radius = max(0.0, float(self.get_parameter("workspace_radius").value))
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.input_timeout_sec = max(0.0, float(self.get_parameter("input_timeout_sec").value))
        self.require_feedback_before_latch = bool(self.get_parameter("require_feedback_before_latch").value)
        self.max_solution_pos_error = max(0.0, float(self.get_parameter("max_solution_pos_error").value))
        self.max_solution_ori_error = max(0.0, float(self.get_parameter("max_solution_ori_error").value))
        self.orientation_mode = str(self.get_parameter("orientation_mode").value).strip()
        if self.orientation_mode not in ("hold_initial", "track_limited", "track_full"):
            self.get_logger().warning(
                f"Unknown orientation_mode='{self.orientation_mode}', falling back to 'hold_initial'"
            )
            self.orientation_mode = "hold_initial"
        self.get_logger().info(
            f"[{self.hand_name}] orientation_mode={self.orientation_mode}"
        )

        # ── Build position mapping from flat YAML array (row-major) ──
        param_name = (
            "position_map_left" if self.hand_name == "left" else "position_map_right"
        )
        flat_map = [float(v) for v in self.get_parameter(param_name).value]
        if len(flat_map) != 9:
            raise ValueError(f"{param_name} must have 9 elements, got {len(flat_map)}")
        self.position_map = np.array(flat_map, dtype=float).reshape(3, 3)
        self.get_logger().info(f"[{self.hand_name}] position_map=\n{self.position_map}")

        # ── Asymmetric workspace limits ─────────────────────────────
        self.max_delta_x_forward = float(self.get_parameter("max_delta_x_forward").value)
        self.max_delta_x_backward = float(self.get_parameter("max_delta_x_backward").value)
        self.max_delta_y_positive = float(self.get_parameter("max_delta_y_positive").value)
        self.max_delta_y_negative = float(self.get_parameter("max_delta_y_negative").value)
        self.max_delta_z_positive = float(self.get_parameter("max_delta_z_positive").value)
        self.max_delta_z_negative = float(self.get_parameter("max_delta_z_negative").value)

        # ── Position filter parameters ──────────────────────────────
        self.target_pos_alpha = max(0.0, min(1.0, float(self.get_parameter("target_pos_alpha").value)))
        self.max_step_pos = max(0.0, float(self.get_parameter("max_step_pos").value))
        self.deadband_pos = max(0.0, float(self.get_parameter("deadband_pos").value))

        # ── Orientation params ─────────────────────────────────────
        self.orientation_gain = max(0.0, float(self.get_parameter("orientation_gain").value))
        self.max_orientation_delta_rad = max(0.0, float(self.get_parameter("max_orientation_delta_rad").value))
        self.max_step_angle = max(0.0, float(self.get_parameter("max_step_angle").value))
        self.target_rot_alpha = max(0.0, min(1.0, float(self.get_parameter("target_rot_alpha").value)))
        self.rot_deadband = max(0.0, float(self.get_parameter("rot_deadband").value))

        # ── Adaptive tremor filter params ──────────────────────────
        self.enable_adaptive_tremor = bool(self.get_parameter("enable_adaptive_tremor_filter").value)
        self.still_pos_threshold = max(0.0, float(self.get_parameter("still_motion_threshold_pos").value))
        self.still_rot_threshold = max(0.0, float(self.get_parameter("still_motion_threshold_rot").value))
        self.still_alpha_scale = max(0.0, min(1.0, float(self.get_parameter("still_filter_alpha_scale").value)))
        self.still_deadband_pos_scale = max(1.0, float(self.get_parameter("still_deadband_pos_scale").value))
        self.still_rot_deadband_scale = max(1.0, float(self.get_parameter("still_rot_deadband_scale").value))

        axis_gain_list = [float(v) for v in self.get_parameter("orientation_axis_gain").value]
        if len(axis_gain_list) != 3:
            raise ValueError(f"orientation_axis_gain must have 3 elements, got {len(axis_gain_list)}")
        self.orientation_axis_gain = np.array(axis_gain_list, dtype=float)

        o_map_param = (
            "orientation_map_left" if self.hand_name == "left" else "orientation_map_right"
        )
        o_flat = [float(v) for v in self.get_parameter(o_map_param).value]
        if len(o_flat) != 9:
            raise ValueError(f"{o_map_param} must have 9 elements, got {len(o_flat)}")
        self.orientation_map_matrix = np.array(o_flat, dtype=float).reshape(3, 3)
        self.get_logger().info(f"[{self.hand_name}] orientation_map=\n{self.orientation_map_matrix}")
        self.get_logger().info(
            f"[{self.hand_name}] orientation params: "
            f"gain={self.orientation_gain}, max_delta_rad={self.max_orientation_delta_rad}, "
            f"max_step_angle={self.max_step_angle}, alpha={self.target_rot_alpha}, "
            f"deadband={self.rot_deadband}, axis_gain={self.orientation_axis_gain}"
        )

        package_path = get_package_share_directory(package_name)
        urdf_path = os.path.join(package_path, urdf_rel)
        self.ik = ArmIK(
            urdf_path=urdf_path,
            package_dirs=[package_path],
            locked_joints=locked_joints,
            ee_parent_joint=ee_parent_joint,
            ee_frame_name=ee_frame_name,
            tool_pre_rot_rpy=tool_pre_rot_rpy,
            tool_translation_xyz=tool_translation_xyz,
            collision_pairs_flat=collision_pairs_flat,
            w_pos=w_pos,
            w_ori=w_ori,
            w_reg=w_reg,
            w_smooth=w_smooth,
            ipopt_max_iter=ipopt_max_iter,
            ipopt_tol=ipopt_tol,
            enable_visualization=enable_visualization,
            viewer_open_browser=viewer_open_browser,
            viewer_model_name=viewer_model_name,
            viewer_target_frame_name=viewer_target_frame_name,
            viewer_axis_length=viewer_axis_length,
            viewer_axis_width=viewer_axis_width,
        )
        dedup_locked = self.ik._deduplicate_locked_joints(locked_joints)
        self.get_logger().info(f"locked_joints(raw)={locked_joints}, locked_joints(dedup)={dedup_locked}")

        output_joint_names = list(self.get_parameter("output_joint_names").value)
        self.output_joint_names = output_joint_names if len(output_joint_names) == self.ik.nq else self.ik.active_joint_names()

        self.pub_joint = self.create_publisher(JointState, pin_joint_status_topic, 10)
        self.pub_collision = self.create_publisher(Bool, f"{pin_joint_status_topic}_collision", 10)
        self.enable_collision_check = enable_collision_check
        self.latest_feedback_q: Optional[np.ndarray] = None
        self.latest_vr_position: Optional[np.ndarray] = None
        self.latest_vr_rotation: Optional[Rotation] = None
        self.initial_vr_position: Optional[np.ndarray] = None
        self.initial_vr_rotation: Optional[Rotation] = None
        self.initial_robot_position: Optional[np.ndarray] = None
        self.initial_robot_rotation: Optional[Rotation] = None
        self.last_target_quat: Optional[np.ndarray] = None
        self.last_pose_time_sec: Optional[float] = None
        self.filtered_target_position: Optional[np.ndarray] = None
        self.filtered_target_rotation: Optional[Rotation] = None
        self.control_active = not bool(self.control_enable_topic)
        self.pending_latch = True
        self.last_log_times = {}

        if pose_stamped_topic:
            self.create_subscription(PoseStamped, pose_stamped_topic, self.pose_stamped_callback, 10)
        if feedback_joint_topic:
            self.create_subscription(JointState, feedback_joint_topic, self.feedback_joint_callback, 10)
        if self.control_enable_topic:
            self.create_subscription(Bool, self.control_enable_topic, self.control_enable_callback, 10)
        if not pose_stamped_topic:
            raise ValueError("pose_stamped_topic cannot be empty.")

        self.get_logger().info(
            f"IK node ready. URDF={urdf_path}, input=({pose_stamped_topic}), "
            f"output={pin_joint_status_topic}, nq={self.ik.nq}, backend={self.ik.backend_name}, "
            f"delta_mapping={self.delta_mapping_enabled}, hand={self.hand_name}, "
            f"orientation_mode={self.orientation_mode}, "
            f"enable_topic={self.control_enable_topic or '<auto>'}"
        )

    def _resolve_hand_name(self, raw_hand_name: str, pose_topic: str, output_topic: str) -> str:
        hand = str(raw_hand_name).strip().lower()
        if hand in ("left", "right"):
            return hand
        if hand not in ("", "auto"):
            raise ValueError("hand_name must be 'left', 'right', or 'auto'")

        hints = f"{self.get_name()} {pose_topic} {output_topic}".lower()
        if "left" in hints:
            return "left"
        if "right" in hints:
            return "right"

        self.get_logger().warning("hand_name=auto could not infer side; defaulting to right")
        return "right"

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_periodically(self, level: str, key: str, period_sec: float, text: str) -> None:
        now_sec = self._now_sec()
        last_time = self.last_log_times.get(key)
        if last_time is not None and now_sec - last_time < period_sec:
            return
        self.last_log_times[key] = now_sec
        logger = self.get_logger()
        if level == "error":
            logger.error(text)
        elif level == "info":
            logger.info(text)
        else:
            logger.warning(text)

    def _extract_active_joint_positions(self, msg: JointState) -> Optional[np.ndarray]:
        if len(msg.position) < self.ik.nq:
            return None

        if msg.name and len(msg.name) == len(msg.position):
            index_by_name = {name: i for i, name in enumerate(msg.name)}
            preferred_names = list(self.output_joint_names[: self.ik.nq])
            if all(name in index_by_name for name in preferred_names):
                return np.array(
                    [msg.position[index_by_name[name]] for name in preferred_names],
                    dtype=float,
                )

            active_names = self.ik.active_joint_names()[: self.ik.nq]
            if all(name in index_by_name for name in active_names):
                return np.array(
                    [msg.position[index_by_name[name]] for name in active_names],
                    dtype=float,
                )

        return np.array(msg.position[: self.ik.nq], dtype=float)

    def feedback_joint_callback(self, msg: JointState) -> None:
        q = self._extract_active_joint_positions(msg)
        if q is None or q.shape[0] != self.ik.nq or not np.all(np.isfinite(q)):
            self._log_periodically(
                "warn",
                "bad_feedback_joint_state",
                1.0,
                f"Ignore invalid feedback JointState: positions={len(msg.position)}, nq={self.ik.nq}",
            )
            return

        self.latest_feedback_q = q
        self.ik.sync_state(q.tolist())

    def control_enable_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled and not self.control_active:
            self.control_active = True
            self._clear_latch()
            self.get_logger().info(f"[{self.hand_name}] control enable rising edge; waiting for latch")
            if self.latest_vr_position is not None and self.latest_vr_rotation is not None:
                self._try_latch(self.latest_vr_position, self.latest_vr_rotation)
        elif not enabled and self.control_active:
            self.control_active = False
            self._clear_latch()
            self.get_logger().info(f"[{self.hand_name}] control disabled; latch cleared")

    def _clear_latch(self) -> None:
        self.initial_vr_position = None
        self.initial_vr_rotation = None
        self.initial_robot_position = None
        self.initial_robot_rotation = None
        self.last_target_quat = None
        self.filtered_target_position = None  # reset filter on re-latch
        self.filtered_target_rotation = None  # reset rotation filter on re-latch
        self.pending_latch = True

    def _has_latch(self) -> bool:
        return (
            self.initial_vr_position is not None
            and self.initial_vr_rotation is not None
            and self.initial_robot_position is not None
            and self.initial_robot_rotation is not None
        )

    def _try_latch(self, vr_position: np.ndarray, vr_rotation: Rotation) -> bool:
        if self.latest_feedback_q is None:
            if self.require_feedback_before_latch:
                self._log_periodically(
                    "warn",
                    "waiting_feedback_for_latch",
                    1.0,
                    f"[{self.hand_name}] cannot latch: waiting for feedback_joint_topic",
                )
                return False
            q_for_fk = self.ik.history_data.copy()
        else:
            q_for_fk = self.latest_feedback_q.copy()

        try:
            robot_pose = self.ik.end_effector_pose(q_for_fk)
        except Exception as exc:
            self._log_periodically(
                "warn",
                "fk_latch_failed",
                1.0,
                f"[{self.hand_name}] cannot latch robot pose from feedback: {exc}",
            )
            return False

        self.initial_vr_position = np.asarray(vr_position, dtype=float).reshape(3).copy()
        self.initial_vr_rotation = Rotation.from_quat(normalize_quat_xyzw(vr_rotation.as_quat()))
        self.initial_robot_position = robot_pose[:3, 3].copy()
        self.initial_robot_rotation = rotation_from_matrix(robot_pose[:3, :3])
        self.last_target_quat = normalize_quat_xyzw(self.initial_robot_rotation.as_quat())
        self.pending_latch = False

        vr_quat = normalize_quat_xyzw(self.initial_vr_rotation.as_quat())
        robot_quat = normalize_quat_xyzw(self.initial_robot_rotation.as_quat())
        self.get_logger().info(
            f"[{self.hand_name}] latched delta teleop: "
            f"robot_pos=({self.initial_robot_position[0]:.4f}, "
            f"{self.initial_robot_position[1]:.4f}, {self.initial_robot_position[2]:.4f}), "
            f"vr_quat=({vr_quat[0]:.4f}, {vr_quat[1]:.4f}, {vr_quat[2]:.4f}, {vr_quat[3]:.4f}), "
            f"robot_quat=({robot_quat[0]:.4f}, {robot_quat[1]:.4f}, {robot_quat[2]:.4f}, {robot_quat[3]:.4f})"
        )

        # ── Health check: warn if robot is near joint limits at latch ──
        lower = np.asarray(self.ik.reduced_robot.model.lowerPositionLimit, dtype=float)
        upper = np.asarray(self.ik.reduced_robot.model.upperPositionLimit, dtype=float)
        margin = 0.15  # rad — joints within this margin of a limit are suspicious
        near_limit = (q_for_fk < lower + margin) | (q_for_fk > upper - margin)
        if np.any(near_limit):
            joint_names = self.ik.active_joint_names()
            bad = [(joint_names[i], float(q_for_fk[i]), float(lower[i]), float(upper[i]))
                   for i in range(len(q_for_fk)) if near_limit[i]]
            self.get_logger().warning(
                f"[{self.hand_name}] latch HEALTH: robot near joint limit! "
                f"margin={margin:.2f}rad, offending joints: "
                + ", ".join(f"{n}(q={q:.3f}, lim=[{lo:.2f},{hi:.2f}])" for n, q, lo, hi in bad)
            )
        else:
            self.get_logger().info(
                f"[{self.hand_name}] latch HEALTH: all joints well within limits "
                f"(margin={margin:.2f}rad)"
            )

        return True

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        try:
            vr_position, vr_rotation = pose_to_position_rotation(msg.pose)
        except Exception as exc:
            self._log_periodically(
                "warn",
                "bad_pose_stamped",
                1.0,
                f"[{self.hand_name}] ignore invalid PoseStamped: {exc}",
            )
            return

        if not self.delta_mapping_enabled:
            self._solve_and_publish(pose_to_mat(msg.pose), stamp=msg.header.stamp)
            return

        now_sec = self._now_sec()
        if (
            not self.control_enable_topic
            and self.last_pose_time_sec is not None
            and self.input_timeout_sec > 0.0
            and now_sec - self.last_pose_time_sec > self.input_timeout_sec
        ):
            self._clear_latch()
            self.get_logger().info(
                f"[{self.hand_name}] input timeout; next PoseStamped will re-latch"
            )
        self.last_pose_time_sec = now_sec

        self.latest_vr_position = vr_position
        self.latest_vr_rotation = vr_rotation

        if not self.control_active:
            return

        if self.pending_latch or not self._has_latch():
            if not self._try_latch(vr_position, vr_rotation):
                return

        target = self._build_delta_target(vr_position, vr_rotation)
        if target is None:
            return

        target_pose, target_quat = target
        self._solve_and_publish(target_pose, stamp=msg.header.stamp, target_quat=target_quat)

    def _build_target_rotation(self, vr_rotation: Rotation) -> Tuple[Rotation, np.ndarray]:
        """Construct target rotation based on orientation_mode."""
        if self.orientation_mode == "hold_initial":
            target_rotation = self.initial_robot_rotation
            target_quat = normalize_quat_xyzw(target_rotation.as_quat())
            return target_rotation, target_quat

        current_vr_rot = vr_rotation
        initial_vr_rot = self.initial_vr_rotation
        delta_R_vr = current_vr_rot * initial_vr_rot.inv()

        if self.orientation_mode == "track_full":
            # Full delta projection through orientation_map
            R_map = self.orientation_map_matrix
            delta_R_robot = rotation_from_matrix(
                R_map @ rotation_to_matrix(delta_R_vr) @ R_map.T
            )
            target_rotation = delta_R_robot * self.initial_robot_rotation
            target_quat = normalize_quat_xyzw(target_rotation.as_quat())
            if self.last_target_quat is not None and float(np.dot(target_quat, self.last_target_quat)) < 0.0:
                target_quat = -target_quat
            return target_rotation, target_quat

        # ── track_limited: rotvec-based safe orientation ──────────
        # 1. VR relative rotation → rotvec
        rotvec_vr = delta_R_vr.as_rotvec()

        # 2. Map to robot frame via orientation_map matrix
        rotvec_robot_raw = self.orientation_map_matrix @ rotvec_vr

        # 3. Per-axis gain
        rotvec_robot = rotvec_robot_raw * self.orientation_axis_gain

        # 4. Overall gain
        rotvec_robot = self.orientation_gain * rotvec_robot

        # 5. Cumulative orientation delta limit
        norm = float(np.linalg.norm(rotvec_robot))
        if norm > self.max_orientation_delta_rad and norm > 1e-9:
            self._log_periodically(
                "warn",
                "ori_delta_clamped",
                0.5,
                f"[{self.hand_name}] orientation delta limited: "
                f"raw_norm={norm:.4f}, limited_norm={self.max_orientation_delta_rad:.4f}, "
                f"max_orientation_delta_rad={self.max_orientation_delta_rad:.4f}",
            )
            rotvec_robot = rotvec_robot / norm * self.max_orientation_delta_rad
            norm = self.max_orientation_delta_rad

        # 6. Deadband
        if norm < self.rot_deadband:
            target_rotation_raw = self.initial_robot_rotation
        else:
            delta_R_robot_limited = Rotation.from_rotvec(rotvec_robot)
            target_rotation_raw = delta_R_robot_limited * self.initial_robot_rotation

        # 7. Rotation filter + per-frame max step
        target_rotation = self._apply_rotation_filter(target_rotation_raw)

        # 8. Quaternion anti-flip
        target_quat = normalize_quat_xyzw(target_rotation.as_quat())
        if self.last_target_quat is not None and float(np.dot(target_quat, self.last_target_quat)) < 0.0:
            self.get_logger().info(f"[{self.hand_name}] ROT: quaternion anti-flip applied")
            target_quat = -target_quat

        # Debug log
        self._log_periodically(
            "info",
            "rot_mapping",
            0.5,
            f"[{self.hand_name}] ROT: mode={self.orientation_mode} "
            f"gain={self.orientation_gain:.2f} "
            f"rotvec_vr=({rotvec_vr[0]:.3f},{rotvec_vr[1]:.3f},{rotvec_vr[2]:.3f}) "
            f"rotvec_robot=({rotvec_robot_raw[0]:.3f},{rotvec_robot_raw[1]:.3f},{rotvec_robot_raw[2]:.3f}) "
            f"target_q=({target_quat[0]:.3f},{target_quat[1]:.3f},{target_quat[2]:.3f},{target_quat[3]:.3f})",
        )

        return target_rotation, target_quat

    def _apply_rotation_filter(self, raw_rot: Rotation) -> Rotation:
        """Low-pass + max-step filter with adaptive tremor suppression."""
        if self.filtered_target_rotation is None:
            self.filtered_target_rotation = raw_rot
            return raw_rot

        delta_R = raw_rot * self.filtered_target_rotation.inv()
        delta_vec = delta_R.as_rotvec()
        angle = float(np.linalg.norm(delta_vec))

        # ── Adaptive tremor filter ────────────────────────────────
        if self.enable_adaptive_tremor and angle < self.still_rot_threshold:
            effective_rot_deadband = self.rot_deadband * self.still_rot_deadband_scale
            effective_rot_alpha = self.target_rot_alpha * self.still_alpha_scale
        else:
            effective_rot_deadband = self.rot_deadband
            effective_rot_alpha = self.target_rot_alpha

        if angle < effective_rot_deadband:
            return self.filtered_target_rotation

        if angle > self.max_step_angle and angle > 1e-9:
            self._log_periodically(
                "warn",
                "rot_step_clamped",
                0.5,
                f"[{self.hand_name}] orientation step limited: "
                f"raw_step={angle:.4f}, limited_step={self.max_step_angle:.4f}, "
                f"max_step_angle={self.max_step_angle:.4f}",
            )
            delta_vec = delta_vec / angle * self.max_step_angle

        delta_vec = effective_rot_alpha * delta_vec

        new_rot = Rotation.from_rotvec(delta_vec) * self.filtered_target_rotation
        self.filtered_target_rotation = new_rot
        return new_rot

    def _build_delta_target(
        self,
        vr_position: np.ndarray,
        vr_rotation: Rotation,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        # ── 1. Compute VR delta ────────────────────────────────────
        delta_vr = np.asarray(vr_position, dtype=float).reshape(3) - self.initial_vr_position
        dx_vr, dy_vr, dz_vr = float(delta_vr[0]), float(delta_vr[1]), float(delta_vr[2])

        # ── 2. Map to robot frame via YAML-configurable matrix ─────
        delta_robot = self.position_map @ np.array([dx_vr, dy_vr, dz_vr], dtype=float)
        target_delta = self.position_scale * delta_robot

        # ── 3. Asymmetric clamp (relative to latch point) ──────────
        target_delta = self._clamp_target_delta(target_delta)

        # ── 4. Raw target position ─────────────────────────────────
        raw_target_pos = self.initial_robot_position + target_delta

        # ── 5. Position filtering (low-pass + step limit + deadband)
        filtered_pos = self._apply_position_filter(raw_target_pos)

        # ── 6. Debug logging ───────────────────────────────────────
        self._log_periodically(
            "info",
            "delta_mapping",
            0.5,
            f"[{self.hand_name}] delta_vr=({dx_vr:.4f},{dy_vr:.4f},{dz_vr:.4f}) "
            f"delta_robot=({delta_robot[0]:.4f},{delta_robot[1]:.4f},{delta_robot[2]:.4f}) "
            f"target_delta=({target_delta[0]:.4f},{target_delta[1]:.4f},{target_delta[2]:.4f}) "
            f"target_pos=({filtered_pos[0]:.4f},{filtered_pos[1]:.4f},{filtered_pos[2]:.4f})",
        )

        # ── 7. Orientation via controllable mode ───────────────────
        target_rotation, target_quat = self._build_target_rotation(vr_rotation)

        return position_rotation_to_mat(filtered_pos, target_rotation), target_quat

    def _clamp_target_delta(self, target_delta: np.ndarray) -> np.ndarray:
        """Asymmetric per-axis clamp relative to latch point."""
        limits = np.array([
            [self.max_delta_x_backward, self.max_delta_x_forward],   # x: [-back, +fwd]
            [self.max_delta_y_negative, self.max_delta_y_positive],  # y: [-neg, +pos]
            [self.max_delta_z_negative, self.max_delta_z_positive],  # z: [-neg, +pos]
        ], dtype=float)
        clamped = target_delta.copy()
        for i in range(3):
            lo, hi = -limits[i, 0], limits[i, 1]
            if clamped[i] < lo or clamped[i] > hi:
                old_val = float(clamped[i])
                clamped[i] = float(np.clip(clamped[i], lo, hi))
                self._log_periodically(
                    "warn",
                    f"clamp_delta_{'xyz'[i]}",
                    0.5,
                    f"[{self.hand_name}] target_delta axis {'xyz'[i]} clamped "
                    f"from {old_val:.4f} to {clamped[i]:.4f} (limit [{lo:.3f},{hi:.3f}])",
                )
        return clamped

    def _apply_position_filter(self, raw_target_pos: np.ndarray) -> np.ndarray:
        """Low-pass + max-step + deadband filter with adaptive tremor suppression."""
        if self.filtered_target_position is None:
            self.filtered_target_position = raw_target_pos.copy()
            return self.filtered_target_position

        delta = raw_target_pos - self.filtered_target_position
        dist = float(np.linalg.norm(delta))

        # ── Adaptive tremor filter ────────────────────────────────
        if self.enable_adaptive_tremor and dist < self.still_pos_threshold:
            effective_deadband = self.deadband_pos * self.still_deadband_pos_scale
            effective_alpha = self.target_pos_alpha * self.still_alpha_scale
        else:
            effective_deadband = self.deadband_pos
            effective_alpha = self.target_pos_alpha

        if dist < effective_deadband:
            return self.filtered_target_position

        if dist > self.max_step_pos:
            delta = delta * (self.max_step_pos / dist)

        limited = self.filtered_target_position + delta
        filtered = (
            effective_alpha * limited
            + (1.0 - effective_alpha) * self.filtered_target_position
        )
        self.filtered_target_position = filtered
        return filtered

    def _restore_ik_seed(self, init_data: np.ndarray, history_data: np.ndarray) -> None:
        self.ik.init_data = init_data.copy()
        self.ik.history_data = history_data.copy()

    def _solve_and_publish(
        self,
        target_pose: np.ndarray,
        stamp,
        target_quat: Optional[np.ndarray] = None,
    ) -> bool:
        prev_init_data = self.ik.init_data.copy()
        prev_history_data = self.ik.history_data.copy()

        try:
            sol_q = self.ik.solve(target_pose)
            if sol_q is None:
                raise RuntimeError("IK solver returned no solution")
            sol_q = np.asarray(sol_q, dtype=float).reshape(-1)
            if sol_q.shape[0] != self.ik.nq or not np.all(np.isfinite(sol_q)):
                raise RuntimeError("IK solver returned invalid joint values")

            pos_err, ori_err = self.ik.pose_error_norms(sol_q, target_pose)
            if (
                self.max_solution_pos_error > 0.0
                and pos_err > self.max_solution_pos_error
            ) or (
                self.max_solution_ori_error > 0.0
                and ori_err > self.max_solution_ori_error
            ):
                self._restore_ik_seed(prev_init_data, prev_history_data)
                target_pos = target_pose[:3, 3]
                if self.initial_robot_position is not None:
                    delta_pos = target_pos - self.initial_robot_position
                    delta_str = f"delta_pos=({delta_pos[0]:.4f}, {delta_pos[1]:.4f}, {delta_pos[2]:.4f})"
                else:
                    delta_str = "delta_pos=N/A"
                self._log_periodically(
                    "error",
                    "ik_residual_too_large",
                    0.5,
                    f"[{self.hand_name}] IK solution rejected: "
                    f"pos_err={pos_err:.4f}m, ori_err={ori_err:.4f}rad, "
                    f"target_pos=({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}), "
                    f"{delta_str}",
                )
                return False

            if self.enable_collision_check:
                col_msg = Bool()
                col_msg.data = self.ik.check_self_collision(sol_q)
                self.pub_collision.publish(col_msg)
                if col_msg.data:
                    self._restore_ik_seed(prev_init_data, prev_history_data)
                    self._log_periodically(
                        "error",
                        "ik_collision_rejected",
                        0.5,
                        f"[{self.hand_name}] IK solution rejected by self-collision check",
                    )
                    return False

            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.name = self.output_joint_names
            joint_msg.position = sol_q.tolist()
            self.pub_joint.publish(joint_msg)

            if target_quat is not None:
                self.last_target_quat = normalize_quat_xyzw(target_quat)
            return True

        except Exception as exc:
            self._restore_ik_seed(prev_init_data, prev_history_data)
            self._log_periodically(
                "error",
                "ik_exception",
                0.5,
                f"[{self.hand_name}] IK solve/send skipped: {exc}",
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ArmIKPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
