#!/usr/bin/env python3
"""Button-only bridge for Quest-to-Piper teleop (IK mode).

Stripped down to:
  - Read Quest buttons (X/Y start/stop, LJ/RJ home, trigger gripper)
  - Publish teleop_enabled for the IK node
  - Handle HOME long-press

All coordinate mapping / delta pose computation has been removed —
that responsibility belongs to arm_ik_pose_node.py in use_ik:=true mode.
"""
import time
from typing import Any, Optional

import rclpy
from oculus_reader import OculusReader
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, SetBool


VALID_BUTTONS = {
    "A",
    "B",
    "X",
    "Y",
    "N",
    "LJ",
    "RJ",
    "LTHU",
    "RTHU",
    "LG",
    "RG",
    "LTR",
    "RTR",
}


class RosOperator(Node):
    def __init__(self):
        super().__init__("pub_delta_pose_node")
        self._declare_parameters()
        self._load_parameters()

        # Only publish to control_joint_topic for gripper commands.
        self.pub_move_j = self.create_publisher(JointState, self.control_joint_topic, 10)
        self.pub_enable_state = (
            self.create_publisher(Bool, self.enable_state_topic, 10)
            if self.enable_state_topic
            else None
        )

        enable_svc = f"/{self.arm_ns}/enable_agx_arm" if self.arm_ns else "enable_agx_arm"
        home_svc = f"/{self.arm_ns}/move_home" if self.arm_ns else "move_home"
        self._enable_cli = self.create_client(SetBool, enable_svc)
        self._home_cli = self.create_client(Empty, home_svc)

        # ── Button-only state ─────────────────────────────────────────
        self.flag = False
        self.last_start_pressed = False
        self.last_stop_pressed = False
        self.filtered_gripper_value = 0.0
        self.last_published_gripper: Optional[float] = None

        self._home_hold_start = 0.0
        self._home_sent = False

        self.oculus_reader = OculusReader()
        time.sleep(0.5)

        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, 1.0),
            self.control_loop,
        )

        self.get_logger().info(
            f"pub_delta_pose BUTTON-ONLY bridge ready ({self.hand_name}). "
            f"start={self.start_button} stop={self.stop_button} home={self.home_button}"
        )

    def _declare_parameters(self):
        self.declare_parameter("control_joint_topic", "/control/joint_states")
        self.declare_parameter("enable_state_topic", "")

        self.declare_parameter("start_button", "A")
        self.declare_parameter("stop_button", "B")
        self.declare_parameter("home_button", "N")
        self.declare_parameter("trigger_axis", "rightTrig")

        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("gripper_max_range", 0.07)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("hand_name", "right")

        self.declare_parameter("gripper_alpha", 0.85)
        self.declare_parameter("gripper_command_deadband", 0.003)
        self.declare_parameter("gripper_publish_rate_hz", 10.0)
        self.declare_parameter("gripper_max_step", 0.002)
        self.declare_parameter("arm_ns", "")
        self.declare_parameter("home_hold_sec", 1.0)

    def _load_parameters(self):
        self.control_joint_topic = str(self.get_parameter("control_joint_topic").value)
        self.enable_state_topic = str(self.get_parameter("enable_state_topic").value).strip()

        self.start_button = self._normalize_button_name(self.get_parameter("start_button").value)
        self.stop_button = self._normalize_button_name(self.get_parameter("stop_button").value)
        self.home_button = self._normalize_button_name(self.get_parameter("home_button").value)
        self.trigger_axis = str(self.get_parameter("trigger_axis").value)

        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_max_range = max(0.0, float(self.get_parameter("gripper_max_range").value))
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.hand_name = str(self.get_parameter("hand_name").value).strip().lower()

        self.gripper_alpha = max(0.0, min(1.0, float(self.get_parameter("gripper_alpha").value)))
        self.gripper_command_deadband = max(
            0.0,
            float(self.get_parameter("gripper_command_deadband").value),
        )
        self.gripper_publish_rate_hz = max(1.0, float(self.get_parameter("gripper_publish_rate_hz").value))
        self.gripper_max_step = max(0.0, float(self.get_parameter("gripper_max_step").value))
        self._last_gripper_publish_time: float = 0.0
        self.home_hold_sec = max(0.1, float(self.get_parameter("home_hold_sec").value))
        self.arm_ns = str(self.get_parameter("arm_ns").value).strip()

    def _normalize_button_name(self, raw: Any) -> str:
        if isinstance(raw, bool):
            return "Y" if raw else "N"
        value = str(raw).strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1].strip()
        value = value.upper()
        if value not in VALID_BUTTONS:
            raise ValueError(f"Invalid button '{raw}', expected one of {sorted(VALID_BUTTONS)}")
        return value

    def _extract_trigger_value(self, buttons: dict) -> float:
        trigger_raw: Any = buttons.get(self.trigger_axis, [0.0])
        if isinstance(trigger_raw, (list, tuple)):
            return float(trigger_raw[0]) if trigger_raw else 0.0
        if isinstance(trigger_raw, (int, float)):
            return float(trigger_raw)
        return 0.0

    def _button_value(self, buttons: dict, button_name: str) -> bool:
        return button_name != "N" and bool(buttons.get(button_name, False))

    def _publish_gripper_if_needed(self, buttons: dict):
        trigger_value = max(0.0, min(self._extract_trigger_value(buttons), 1.0))
        target_value = trigger_value * self.gripper_max_range
        self.filtered_gripper_value = (
            self.gripper_alpha * target_value
            + (1.0 - self.gripper_alpha) * self.filtered_gripper_value
        )

        if (
            self.last_published_gripper is not None
            and abs(self.filtered_gripper_value - self.last_published_gripper)
            < self.gripper_command_deadband
        ):
            return

        # ── Rate limit: max gripper_publish_rate_hz ────────────────
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        min_interval = 1.0 / max(self.gripper_publish_rate_hz, 1.0)
        if now_sec - self._last_gripper_publish_time < min_interval:
            return

        # ── Max step limit: prevent large jumps ────────────────────
        final_value = float(self.filtered_gripper_value)
        if self.last_published_gripper is not None:
            step = final_value - self.last_published_gripper
            if abs(step) > self.gripper_max_step:
                step = self.gripper_max_step if step > 0 else -self.gripper_max_step
                final_value = self.last_published_gripper + step
                self.filtered_gripper_value = final_value

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.gripper_joint_name]
        msg.position = [final_value]
        self.pub_move_j.publish(msg)
        self.last_published_gripper = final_value
        self._last_gripper_publish_time = now_sec

    def _stop_teleop(self):
        if self.flag:
            self.get_logger().info(f"[{self.hand_name}] stop teleop")
        self.flag = False

    def _publish_enable_state(self):
        if self.pub_enable_state is None:
            return
        msg = Bool()
        msg.data = bool(self.flag)
        self.pub_enable_state.publish(msg)

    def _do_home(self):
        # ── Step 1: Cut off topic control flow ──────────────────────────
        # Immediately publish teleop_enabled=False to force the IK node to
        # stop computing and publishing /control/joint_states.  This
        # prevents the driver from receiving concurrent topic commands
        # while processing the HOME service request.
        self._stop_teleop()
        if self.pub_enable_state is not None:
            msg = Bool()
            msg.data = False
            self.pub_enable_state.publish(msg)
        self.get_logger().info(
            f"[{self.hand_name}] HOME: teleop control flow cut — "
            f"teleop_enabled=False published"
        )

        # ── Step 2: Wait for move_home service ──────────────────────────
        if not self._home_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f"[{self.hand_name}] HOME: /{self.arm_ns}/move_home "
                f"service not available after 2 s timeout — aborting HOME"
            )
            return

        # ── Step 3: Quick enable attempt (non-blocking) ─────────────────
        # The arm must be enabled for move_home to succeed.  Fire-and-forget.
        if self._enable_cli.service_is_ready():
            self._enable_cli.call_async(SetBool.Request(data=True))

        # ── Step 4: Call the HOME service ───────────────────────────────
        self.get_logger().info(
            f"[{self.hand_name}] HOME: calling /{self.arm_ns}/move_home"
        )
        future = self._home_cli.call_async(Empty.Request())

        # ── Step 5: Log the service result ──────────────────────────────
        def _home_done_callback(fut):
            if fut.result() is not None:
                self.get_logger().info(
                    f"[{self.hand_name}] HOME: /{self.arm_ns}/move_home "
                    f"service call succeeded"
                )
            else:
                self.get_logger().error(
                    f"[{self.hand_name}] HOME: /{self.arm_ns}/move_home "
                    f"service call returned None / failed"
                )

        future.add_done_callback(_home_done_callback)

        # ── State maintained ────────────────────────────────────────────
        # flag=False from _stop_teleop().  Teleop stays disabled until the
        # user presses the start button again to begin a new control cycle.
        self.get_logger().info(
            f"[{self.hand_name}] HOME: teleop stopped — "
            f"press start button to begin new control cycle"
        )

    def _handle_home_button(self, buttons: dict, now_sec: float):
        home_pressed = self._button_value(buttons, self.home_button)

        # ── Raw button diagnostics: log EVERY press immediately ─────────
        if home_pressed:
            hold_dur = now_sec - self._home_hold_start if self._home_hold_start > 0.0 else 0.0
            self.get_logger().info(
                f"[{self.hand_name}] [Raw Button] {self.home_button} is pressed! "
                f"hold_duration={hold_dur:.2f}s"
            )

        if home_pressed:
            if self._home_hold_start == 0.0:
                self._home_hold_start = now_sec
            elif now_sec - self._home_hold_start >= self.home_hold_sec and not self._home_sent:
                self._do_home()
                self._home_sent = True
        else:
            self._home_hold_start = 0.0
            self._home_sent = False

    def control_loop(self):
        _, buttons = self.oculus_reader.get_transformations_and_buttons()
        self._publish_gripper_if_needed(buttons)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._handle_home_button(buttons, now_sec)

        start_pressed = self._button_value(buttons, self.start_button)
        stop_pressed = self._button_value(buttons, self.stop_button)

        if start_pressed and not self.last_start_pressed:
            self.flag = True
            self.get_logger().info(
                f"[{self.hand_name}] start pressed — teleop_enabled=True, "
                f"IK node will latch on next OpenXR frame"
            )

        if stop_pressed and not self.last_stop_pressed:
            self._stop_teleop()

        self.last_start_pressed = start_pressed
        self.last_stop_pressed = stop_pressed
        self._publish_enable_state()

    def destroy_node(self):
        self.oculus_reader.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RosOperator()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
