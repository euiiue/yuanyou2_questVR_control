#!/usr/bin/env python3
"""Map Quest WebXR controller poses to Piper ROS PosCmd commands.

The node is deliberately conservative:
- It always publishes preview PosCmd topics.
- It publishes real /left/pos_cmd and /right/pos_cmd only when publish_real is true.
- If require_deadman is true, each arm must hold its deadman button to publish real commands.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import rospy
from geometry_msgs.msg import PoseStamped
from piper_msgs.msg import PosCmd
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


@dataclass
class ArmState:
    pose: PoseStamped | None = None
    joy: Joy | None = None
    origin_xyz: tuple[float, float, float] | None = None
    current_xyz: list[float] | None = None
    current_rpy: list[float] | None = None
    last_recenter_pressed: bool = False
    initialized: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)


class QuestVrPiperAdapter:
    def __init__(self) -> None:
        rospy.init_node("quest_vr_piper_adapter", anonymous=False)

        self.publish_real = bool(rospy.get_param("~publish_real", False))
        self.require_deadman = bool(rospy.get_param("~require_deadman", True))
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.position_scale = float(rospy.get_param("~position_scale", 1.0))
        self.rotation_scale = float(rospy.get_param("~rotation_scale", 1.0))
        self.position_alpha = self._clamp(float(rospy.get_param("~position_alpha", 0.25)), 0.01, 1.0)
        self.rotation_alpha = self._clamp(float(rospy.get_param("~rotation_alpha", 0.20)), 0.01, 1.0)

        self.deadman_button_index = int(rospy.get_param("~deadman_button_index", 1))
        self.recenter_button_index = int(rospy.get_param("~recenter_button_index", 3))
        self.trigger_button_index = int(rospy.get_param("~trigger_button_index", 0))
        self.trigger_axis_index = int(rospy.get_param("~trigger_axis_index", 4))

        self.use_controller_orientation = bool(rospy.get_param("~use_controller_orientation", False))

        self.left_home = self._param_vec("~left_home_xyz", [0.30, 0.22, 0.30])
        self.right_home = self._param_vec("~right_home_xyz", [0.30, -0.22, 0.30])
        self.left_fixed_rpy = self._param_vec("~left_fixed_rpy", [0.0, 0.0, 0.0])
        self.right_fixed_rpy = self._param_vec("~right_fixed_rpy", [0.0, 0.0, 0.0])

        self.x_min = float(rospy.get_param("~x_min", 0.15))
        self.x_max = float(rospy.get_param("~x_max", 0.55))
        self.y_min = float(rospy.get_param("~y_min", -0.45))
        self.y_max = float(rospy.get_param("~y_max", 0.45))
        self.z_min = float(rospy.get_param("~z_min", 0.02))
        self.z_max = float(rospy.get_param("~z_max", 0.60))
        self.max_delta_per_tick = float(rospy.get_param("~max_delta_per_tick", 0.015))

        self.gripper_open = float(rospy.get_param("~gripper_open", 0.07))
        self.gripper_closed = float(rospy.get_param("~gripper_closed", 0.0))

        self.left = ArmState()
        self.right = ArmState()

        self.left_preview_pub = rospy.Publisher("/quest_vr_piper/left_pos_cmd", PosCmd, queue_size=1)
        self.right_preview_pub = rospy.Publisher("/quest_vr_piper/right_pos_cmd", PosCmd, queue_size=1)
        self.status_pub = rospy.Publisher("/quest_vr_piper/status", String, queue_size=3)

        self.left_real_pub = (
            rospy.Publisher("/left/pos_cmd", PosCmd, queue_size=1)
            if self.publish_real
            else None
        )
        self.right_real_pub = (
            rospy.Publisher("/right/pos_cmd", PosCmd, queue_size=1)
            if self.publish_real
            else None
        )

        rospy.Subscriber("/quest_vr/left/pose", PoseStamped, self._left_pose_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/quest_vr/right/pose", PoseStamped, self._right_pose_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/quest_vr/left/joy", Joy, self._left_joy_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/quest_vr/right/joy", Joy, self._right_joy_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/quest_vr/piper_enable", Bool, self._enable_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._timer_cb)

        rospy.loginfo("QuestVrPiperAdapter started")
        rospy.loginfo("publish_real=%s require_deadman=%s enabled=%s", self.publish_real, self.require_deadman, self.enabled)
        rospy.loginfo("left_home=%s right_home=%s scale=%.3f", self.left_home, self.right_home, self.position_scale)
        rospy.logwarn("Real Piper output is %s", "ENABLED" if self.publish_real else "DISABLED; preview only")

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _param_vec(name: str, default: list[float]) -> list[float]:
        values = rospy.get_param(name, default)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return [float(v) for v in values]

    def _left_pose_cb(self, msg: PoseStamped) -> None:
        with self.left.lock:
            self.left.pose = msg

    def _right_pose_cb(self, msg: PoseStamped) -> None:
        with self.right.lock:
            self.right.pose = msg

    def _left_joy_cb(self, msg: Joy) -> None:
        with self.left.lock:
            self.left.joy = msg

    def _right_joy_cb(self, msg: Joy) -> None:
        with self.right.lock:
            self.right.joy = msg

    def _enable_cb(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        rospy.logwarn("Quest Piper adapter enabled=%s", self.enabled)

    def _button_pressed(self, joy: Joy | None, index: int) -> bool:
        return bool(joy and index >= 0 and index < len(joy.buttons) and joy.buttons[index])

    def _button_value(self, joy: Joy | None, button_index: int, axis_index: int) -> float:
        if joy and axis_index >= 0 and axis_index < len(joy.axes):
            return self._clamp(float(joy.axes[axis_index]), 0.0, 1.0)
        if self._button_pressed(joy, button_index):
            return 1.0
        return 0.0

    def _extract_xyz(self, pose: PoseStamped) -> tuple[float, float, float]:
        p = pose.pose.position
        return (float(p.x), float(p.y), float(p.z))

    def _webxr_delta_to_piper(self, current: tuple[float, float, float], origin: tuple[float, float, float]) -> tuple[float, float, float]:
        # WebXR local-floor is right-handed: +X right, +Y up, -Z forward.
        dx = current[0] - origin[0]
        dy = current[1] - origin[1]
        dz = current[2] - origin[2]
        return (
            -dz * self.position_scale,
            -dx * self.position_scale,
            dy * self.position_scale,
        )

    def _target_xyz(self, state: ArmState, home: list[float]) -> list[float] | None:
        if state.pose is None:
            return None
        xyz = self._extract_xyz(state.pose)
        if state.origin_xyz is None:
            state.origin_xyz = xyz
            state.initialized = True
            rospy.loginfo("Captured Quest origin: %s", xyz)
        delta = self._webxr_delta_to_piper(xyz, state.origin_xyz)
        return [
            self._clamp(home[0] + delta[0], self.x_min, self.x_max),
            self._clamp(home[1] + delta[1], self.y_min, self.y_max),
            self._clamp(home[2] + delta[2], self.z_min, self.z_max),
        ]

    def _target_rpy(self, state: ArmState, fixed_rpy: list[float]) -> list[float]:
        if not self.use_controller_orientation or state.pose is None:
            return list(fixed_rpy)
        q = state.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w], axes="sxyz")
        return [
            self._clamp(-roll * self.rotation_scale, -math.pi, math.pi),
            self._clamp(-pitch * self.rotation_scale, -math.pi / 2.0, math.pi / 2.0),
            self._clamp(yaw * self.rotation_scale, -math.pi, math.pi),
        ]

    def _smooth_vec(self, current: list[float] | None, target: list[float], alpha: float, max_step: float | None = None) -> list[float]:
        if current is None:
            return list(target)
        out = []
        for c, t in zip(current, target):
            step = (t - c) * alpha
            if max_step is not None:
                step = self._clamp(step, -max_step, max_step)
            out.append(c + step)
        return out

    def _build_cmd(self, xyz: list[float], rpy: list[float], gripper: float) -> PosCmd:
        msg = PosCmd()
        msg.x, msg.y, msg.z = [float(v) for v in xyz]
        msg.roll, msg.pitch, msg.yaw = [float(v) for v in rpy]
        msg.gripper = float(self._clamp(gripper, self.gripper_closed, self.gripper_open))
        msg.mode1 = 0
        msg.mode2 = 0
        return msg

    def _maybe_recenter(self, state: ArmState, side: str) -> None:
        pressed = self._button_pressed(state.joy, self.recenter_button_index)
        if pressed and not state.last_recenter_pressed and state.pose is not None:
            state.origin_xyz = self._extract_xyz(state.pose)
            state.current_xyz = None
            state.current_rpy = None
            rospy.logwarn("Recentered %s Quest origin: %s", side, state.origin_xyz)
        state.last_recenter_pressed = pressed

    def _process_arm(
        self,
        state: ArmState,
        side: str,
        home: list[float],
        fixed_rpy: list[float],
        preview_pub: rospy.Publisher,
        real_pub: rospy.Publisher | None,
    ) -> tuple[bool, str]:
        with state.lock:
            self._maybe_recenter(state, side)
            target_xyz = self._target_xyz(state, home)
            if target_xyz is None:
                return False, "no_pose"
            target_rpy = self._target_rpy(state, fixed_rpy)
            trigger = self._button_value(state.joy, self.trigger_button_index, self.trigger_axis_index)
            gripper = self.gripper_open - trigger * (self.gripper_open - self.gripper_closed)
            deadman = self._button_pressed(state.joy, self.deadman_button_index)

            state.current_xyz = self._smooth_vec(state.current_xyz, target_xyz, self.position_alpha, self.max_delta_per_tick)
            state.current_rpy = self._smooth_vec(state.current_rpy, target_rpy, self.rotation_alpha, None)
            cmd = self._build_cmd(state.current_xyz, state.current_rpy, gripper)

        preview_pub.publish(cmd)
        real_ok = self.enabled and self.publish_real and (deadman or not self.require_deadman)
        if real_ok and real_pub is not None:
            real_pub.publish(cmd)
        return real_ok, "real" if real_ok else "preview"

    def _timer_cb(self, _event: rospy.TimerEvent) -> None:
        left_real, left_state = self._process_arm(
            self.left, "left", self.left_home, self.left_fixed_rpy, self.left_preview_pub, self.left_real_pub
        )
        right_real, right_state = self._process_arm(
            self.right, "right", self.right_home, self.right_fixed_rpy, self.right_preview_pub, self.right_real_pub
        )
        status = f"enabled={self.enabled} publish_real={self.publish_real} left={left_state} right={right_state}"
        self.status_pub.publish(status)
        if self.publish_real and not (left_real or right_real):
            rospy.loginfo_throttle(2.0, "Waiting for deadman buttons before publishing real Piper commands")


def main() -> None:
    QuestVrPiperAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
