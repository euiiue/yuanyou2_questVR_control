import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    oculus_reader_pkg_dir = get_package_share_directory("oculus_reader")
    agx_arm_ctrl_pkg_dir = get_package_share_directory("agx_arm_ctrl")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Whether to launch RViz windows.",
    )
    left_can_port_arg = DeclareLaunchArgument(
        "left_can_port",
        default_value="can_left",
        description="CAN interface for the left arm.",
    )
    right_can_port_arg = DeclareLaunchArgument(
        "right_can_port",
        default_value="can_right",
        description="CAN interface for the right arm.",
    )
    use_ik_arg = DeclareLaunchArgument(
        "use_ik",
        default_value="false",
        choices=["true", "false"],
        description="Whether to route VR poses through arm_ik_pose_node before control.",
    )
    auto_enable_arg = DeclareLaunchArgument(
        "auto_enable",
        default_value="true",
        choices=["true", "false"],
        description="Automatically enable both arms.",
    )
    control_enabled_arg = DeclareLaunchArgument(
        "control_enabled",
        default_value="true",
        choices=["true", "false"],
        description="Whether both arm drivers accept /control/* commands.",
    )

    arm_ik_param_file = os.path.join(
        oculus_reader_pkg_dir, "config", "arm_ik_pose_node.piper.yaml"
    )
    button_a = ParameterValue("A", value_type=str)
    button_b = ParameterValue("B", value_type=str)
    button_x = ParameterValue("X", value_type=str)
    button_y = ParameterValue("Y", value_type=str)
    left_home_positions = [0.403, 2.015, -1.562, 0.064, -0.062, 0.247]
    right_home_positions = [-0.079, 2.084, -1.513, 1.743, 0.141, -0.049]
    teleop_filter_params = {
        "input_pos_alpha": 0.30,
        "input_rot_alpha": 0.45,
        "alpha_pos": 0.55,
        "alpha_rot": 0.55,
        "max_step_pos": 0.012,
        "max_step_angle": 0.16,
        "deadband_pos": 0.0015,
        "rot_deadband": 0.02,
        "output_gain": 1.0,
        "max_anchor_delta_xyz": [0.45, 0.45, 0.35],
        "target_pos_hold_deadband": 0.002,
        "reset_settle_sec": 0.25,
        "reset_delta_pos_threshold": 0.015,
        "reset_delta_rot_threshold": 0.12,
        "gripper_alpha": 0.85,
        "gripper_command_deadband": 0.003,
    }

    # 1) ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py ...
    left_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("left_can_port"),
            "namespace": "left_arm",
            "arm_type": "piper",
            "auto_enable": LaunchConfiguration("auto_enable"),
            "control_enabled": LaunchConfiguration("control_enabled"),
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "home_joint_positions": str(left_home_positions),
            "control": "false",
            "fast_mode": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    right_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("right_can_port"),
            "namespace": "right_arm",
            "arm_type": "piper",
            "auto_enable": LaunchConfiguration("auto_enable"),
            "control_enabled": LaunchConfiguration("control_enabled"),
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "home_joint_positions": str(right_home_positions),
            "control": "false",
            "fast_mode": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    # 2) ros2 run oculus_reader arm_ik_pose_node.py --ros-args --params-file ...
    left_arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="left_arm_ik_pose_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/left_openxr_pose",
                "feedback_joint_topic": "/left_arm/feedback/joint_states",
                "pin_joint_status_topic": "/left_arm/internal/ik_joint_states",
                "hand_name": "left",
                "control_enable_topic": "/left_arm/teleop_enabled",
                "delta_mapping_enabled": True,
                "orientation_mode": "track_limited",
            },
        ],
    )

    right_arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="right_arm_ik_pose_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/right_openxr_pose",
                "feedback_joint_topic": "/right_arm/feedback/joint_states",
                "pin_joint_status_topic": "/right_arm/internal/ik_joint_states",
                "hand_name": "right",
                "control_enable_topic": "/right_arm/teleop_enabled",
                "delta_mapping_enabled": True,
                "orientation_mode": "track_limited",
            },
        ],
    )

    # 3) ros2 run oculus_reader pub_pose.py --ros-args -p ros_to_arm_rpy:=...
    pub_pose_node = Node(
        package="oculus_reader",
        executable="pub_pose.py",
        name="pub_pose_node",
        output="screen",
        # pika frame to arm ee frame
        arguments=["--ros-args", "-p", "ros_to_arm_rpy:=[0.0, 1.5708, 0.0]"],
        parameters=[
            {
                "publish_raw_openxr_pose": ParameterValue(
                    LaunchConfiguration("use_ik"), value_type=bool
                ),
            }
        ],
    )

    # 4) ros2 run oculus_reader pub_delta_pose.py
    # Left hand -> left arm delta pose
    left_pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="left_pub_delta_pose_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "hand_name": "left",
                "handle_pose_topic": "/left_handle_pose",
                "feedback_tcp_pose_topic": "/left_arm/feedback/tcp_pose",
                "delta_pose_topic": "/left_delta_pose",
                "control_joint_topic": "/left_arm/internal/gripper_joint_states",
                "enable_state_topic": "/left_arm/teleop_enabled",
                "start_button": button_x,
                "stop_button": button_y,
                "trigger_axis": "leftTrig",
                "gripper_max_range": 0.07,
                **teleop_filter_params,
                "arm_ns": "left_arm",
                "home_button": ParameterValue("LJ", value_type=str),
                "home_hold_sec": 1.0,
                "prep_home_joint_names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
                "prep_home_positions": left_home_positions,
            }
        ],
    )
    left_pub_delta_pose_direct_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="left_pub_delta_pose_direct_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "hand_name": "left",
                "handle_pose_topic": "/left_handle_pose",
                "feedback_tcp_pose_topic": "/left_arm/feedback/tcp_pose",
                "delta_pose_topic": "/left_arm/control/move_p",
                "control_joint_topic": "/left_arm/control/joint_states",
                "start_button": button_x,
                "stop_button": button_y,
                "trigger_axis": "leftTrig",
                "gripper_max_range": 0.07,
                **teleop_filter_params,
                "arm_ns": "left_arm",
                "home_button": ParameterValue("LJ", value_type=str),
                "home_hold_sec": 1.0,
                "prep_home_joint_names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
                "prep_home_positions": left_home_positions,
            }
        ],
    )

    # Right hand -> right arm delta pose
    right_pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="right_pub_delta_pose_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "hand_name": "right",
                "handle_pose_topic": "/right_handle_pose",
                "feedback_tcp_pose_topic": "/right_arm/feedback/tcp_pose",
                "delta_pose_topic": "/right_delta_pose",
                "control_joint_topic": "/right_arm/internal/gripper_joint_states",
                "enable_state_topic": "/right_arm/teleop_enabled",
                "start_button": button_a,
                "stop_button": button_b,
                "trigger_axis": "rightTrig",
                "gripper_max_range": 0.07,
                **teleop_filter_params,
                "arm_ns": "right_arm",
                "home_button": ParameterValue("RJ", value_type=str),
                "home_hold_sec": 1.0,
                "prep_home_joint_names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
                "prep_home_positions": right_home_positions,
            }
        ],
    )
    right_pub_delta_pose_direct_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="right_pub_delta_pose_direct_node",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "hand_name": "right",
                "handle_pose_topic": "/right_handle_pose",
                "feedback_tcp_pose_topic": "/right_arm/feedback/tcp_pose",
                "delta_pose_topic": "/right_arm/control/move_p",
                "control_joint_topic": "/right_arm/control/joint_states",
                "start_button": button_a,
                "stop_button": button_b,
                "trigger_axis": "rightTrig",
                "gripper_max_range": 0.07,
                **teleop_filter_params,
                "arm_ns": "right_arm",
                "home_button": ParameterValue("RJ", value_type=str),
                "home_hold_sec": 1.0,
                "prep_home_joint_names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
                "prep_home_positions": right_home_positions,
            }
        ],
    )

    # ── Joint command mux nodes (IK mode only) ─────────────────────────
    # Single publisher per arm → no multi-publisher conflict on /control/joint_states
    left_joint_command_mux_node = Node(
        package="oculus_reader",
        executable="joint_command_mux_node.py",
        name="left_joint_command_mux_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "arm_input_topic": "/left_arm/internal/ik_joint_states",
                "gripper_input_topic": "/left_arm/internal/gripper_joint_states",
                "output_topic": "/left_arm/control/joint_states",
                "arm_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                "gripper_joint_name": "gripper",
                "publish_rate_hz": 30.0,
                "require_arm_before_publish": True,
                "publish_full_state_when_gripper_known": True,
                "gripper_alpha": 0.35,
                "gripper_command_deadband": 0.004,
                "gripper_max_step": 0.0025,
                "gripper_max_step_open": 0.0035,
                "gripper_max_step_close": 0.0025,
                "gripper_open_direction": 1,
            },
        ],
    )

    right_joint_command_mux_node = Node(
        package="oculus_reader",
        executable="joint_command_mux_node.py",
        name="right_joint_command_mux_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ik")),
        parameters=[
            {
                "arm_input_topic": "/right_arm/internal/ik_joint_states",
                "gripper_input_topic": "/right_arm/internal/gripper_joint_states",
                "output_topic": "/right_arm/control/joint_states",
                "arm_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
                "gripper_joint_name": "gripper",
                "publish_rate_hz": 30.0,
                "require_arm_before_publish": True,
                "publish_full_state_when_gripper_known": True,
                "gripper_alpha": 0.35,
                "gripper_command_deadband": 0.004,
                "gripper_max_step": 0.0025,
                "gripper_max_step_open": 0.0035,
                "gripper_max_step_close": 0.0025,
                "gripper_open_direction": 1,
            },
        ],
    )

    # 5) ros2 run rviz2 rviz2 --ros-args -p config:=...
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=['-d', os.path.join(get_package_share_directory('oculus_reader'), 'config', 'oculus_reader.rviz')]
    )
    
    return LaunchDescription(
        [
            use_rviz_arg,
            left_can_port_arg,
            right_can_port_arg,
            use_ik_arg,
            auto_enable_arg,
            control_enabled_arg,
            left_arm_driver_launch,
            right_arm_driver_launch,
            left_arm_ik_pose_node,
            right_arm_ik_pose_node,
            left_pub_delta_pose_node,
            right_pub_delta_pose_node,
            left_pub_delta_pose_direct_node,
            right_pub_delta_pose_direct_node,
            left_joint_command_mux_node,
            right_joint_command_mux_node,
            pub_pose_node,
            rviz_node,
        ]
    )
