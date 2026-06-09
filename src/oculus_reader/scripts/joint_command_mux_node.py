#!/usr/bin/env python3
"""Joint command multiplexer — single publisher for /control/joint_states.

Merges arm IK output (6 joints) with gripper command (1 joint) so the
underlying driver topic has exactly one publisher.  Applies low-pass,
deadband, and max-step filtering to the gripper to suppress jitter.
"""
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointCommandMuxNode(Node):
    def __init__(self):
        super().__init__("joint_command_mux_node")
        self._declare_parameters()
        self._load_parameters()

        # ── State ──────────────────────────────────────────────────
        self.latest_arm_positions: Optional[List[float]] = None
        self.latest_arm_stamp: Optional[float] = None
        self.latest_gripper_raw: Optional[float] = None
        self.filtered_gripper_value: float = 0.0
        self.last_published_gripper: Optional[float] = None
        self.last_gripper_update_time: float = 0.0
        self._start_time: float = 0.0

        # ── Subscriptions ──────────────────────────────────────────
        self.create_subscription(
            JointState, self.arm_input_topic, self._arm_callback, 10
        )
        self.create_subscription(
            JointState, self.gripper_input_topic, self._gripper_callback, 10
        )

        # ── Publisher ──────────────────────────────────────────────
        self._pub = self.create_publisher(JointState, self.output_topic, 10)

        # ── Timer ──────────────────────────────────────────────────
        self._timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self._timer_publish,
        )

        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"joint_command_mux ready: "
            f"arm_in={self.arm_input_topic} gripper_in={self.gripper_input_topic} "
            f"out={self.output_topic} rate={self.publish_rate_hz}Hz"
        )

    def _declare_parameters(self):
        self.declare_parameter("arm_input_topic", "/right_arm/internal/ik_joint_states")
        self.declare_parameter("gripper_input_topic", "/right_arm/internal/gripper_joint_states")
        self.declare_parameter("output_topic", "/right_arm/control/joint_states")
        self.declare_parameter("arm_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("require_arm_before_publish", True)
        self.declare_parameter("gripper_alpha", 0.35)
        self.declare_parameter("gripper_command_deadband", 0.004)
        self.declare_parameter("gripper_max_step", 0.0025)
        self.declare_parameter("gripper_max_step_open", 0.0035)
        self.declare_parameter("gripper_max_step_close", 0.0025)
        self.declare_parameter("gripper_open_direction", 1)
        self.declare_parameter("publish_full_state_when_gripper_known", True)

    def _load_parameters(self):
        self.arm_input_topic = str(self.get_parameter("arm_input_topic").value)
        self.gripper_input_topic = str(self.get_parameter("gripper_input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.arm_joint_names = list(self.get_parameter("arm_joint_names").value)
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.require_arm_before_publish = bool(self.get_parameter("require_arm_before_publish").value)
        self.gripper_alpha = max(0.0, min(1.0, float(self.get_parameter("gripper_alpha").value)))
        self.gripper_command_deadband = max(0.0, float(self.get_parameter("gripper_command_deadband").value))
        self.gripper_max_step = max(0.0, float(self.get_parameter("gripper_max_step").value))
        self.gripper_max_step_open = max(0.0, float(self.get_parameter("gripper_max_step_open").value))
        self.gripper_max_step_close = max(0.0, float(self.get_parameter("gripper_max_step_close").value))
        self.gripper_open_direction = float(self.get_parameter("gripper_open_direction").value)
        if self.gripper_open_direction not in (-1.0, 1.0):
            self.gripper_open_direction = 1.0
        self.publish_full_state = bool(self.get_parameter("publish_full_state_when_gripper_known").value)

    def _arm_callback(self, msg: JointState):
        # Extract arm positions by name order
        if msg.name and len(msg.name) == len(msg.position):
            idx_map = {n: i for i, n in enumerate(msg.name)}
            try:
                self.latest_arm_positions = [
                    float(msg.position[idx_map[name]]) for name in self.arm_joint_names
                ]
            except KeyError:
                return
        elif len(msg.position) >= len(self.arm_joint_names):
            self.latest_arm_positions = [
                float(msg.position[i]) for i in range(len(self.arm_joint_names))
            ]
        else:
            return
        self.latest_arm_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _gripper_callback(self, msg: JointState):
        if msg.name and self.gripper_joint_name in msg.name:
            idx = msg.name.index(self.gripper_joint_name)
            self.latest_gripper_raw = float(msg.position[idx])
        elif len(msg.position) >= 1:
            self.latest_gripper_raw = float(msg.position[0])
        else:
            return
        self.last_gripper_update_time = self.get_clock().now().nanoseconds * 1e-9

    def _timer_publish(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.require_arm_before_publish and self.latest_arm_positions is None:
            return

        # ── Gripper filtering ─────────────────────────────────────
        gripper_value: Optional[float] = None
        if self.latest_gripper_raw is not None:
            raw = self.latest_gripper_raw
            # Low-pass
            self.filtered_gripper_value = (
                self.gripper_alpha * raw
                + (1.0 - self.gripper_alpha) * self.filtered_gripper_value
            )
            # Deadband
            if self.last_published_gripper is not None:
                if abs(self.filtered_gripper_value - self.last_published_gripper) < self.gripper_command_deadband:
                    gripper_value = self.last_published_gripper
                else:
                    # Split open/close step limit
                    step = self.filtered_gripper_value - self.last_published_gripper
                    is_opening = (step * self.gripper_open_direction) > 0.0
                    step_limit = (
                        self.gripper_max_step_open if is_opening
                        else self.gripper_max_step_close
                    )
                    if abs(step) > step_limit:
                        step = step_limit if step > 0 else -step_limit
                        self.filtered_gripper_value = self.last_published_gripper + step
                    gripper_value = self.filtered_gripper_value
            else:
                gripper_value = self.filtered_gripper_value

            if gripper_value is not None:
                self.last_published_gripper = gripper_value

        # ── Build output JointState ───────────────────────────────
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        if self.latest_arm_positions is not None:
            if self.publish_full_state and gripper_value is not None:
                msg.name = self.arm_joint_names + [self.gripper_joint_name]
                msg.position = list(self.latest_arm_positions) + [float(gripper_value)]
            else:
                msg.name = list(self.arm_joint_names)
                msg.position = list(self.latest_arm_positions)
        else:
            return

        self._pub.publish(msg)

        # ── Periodic status log ────────────────────────────────────
        arm_age = now_sec - self.latest_arm_stamp if self.latest_arm_stamp else -1.0
        gripper_age = (
            now_sec - self.last_gripper_update_time
            if self.last_gripper_update_time > 0.0
            else -1.0
        )
        if int(now_sec) != int(getattr(self, "_last_status_sec", 0)):
            self._last_status_sec = int(now_sec)
            self.get_logger().info(
                f"mux {self.output_topic}: "
                f"arm_age={arm_age:.3f}s gripper_age={gripper_age:.3f}s "
                f"gripper_raw={self.latest_gripper_raw} "
                f"gripper_filt={gripper_value} "
                f"arm_ok={self.latest_arm_positions is not None} "
                f"gripper_ok={self.latest_gripper_raw is not None}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
