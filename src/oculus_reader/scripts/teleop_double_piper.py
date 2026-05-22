#!/usr/bin/env python3
import rospy
import tf2_ros
import rospkg
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, euler_from_quaternion

import casadi
import meshcat.geometry as mg
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
from oculus_reader import OculusReader

import os
import numpy as np
import math
import ast

from tools import MATHTOOLS
from piper_control import PIPER

from geometry_msgs.msg import PoseStamped

def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]

def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix

def calc_pose_incre(base_pose, pose_data):
    begin_matrix = create_transformation_matrix(base_pose[0], base_pose[1], base_pose[2],
                                                base_pose[3], base_pose[4], base_pose[5])
    zero_matrix = create_transformation_matrix(0.19, 0.0, 0.2, 0, 0, 0)
    end_matrix = create_transformation_matrix(pose_data[0], pose_data[1], pose_data[2],
                                            pose_data[3], pose_data[4], pose_data[5])
    result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
    xyzrpy = matrix_to_xyzrpy(result_matrix)
    return xyzrpy

def calc_pose_delta_matrix(base_pose, pose_data):
    begin_matrix = create_transformation_matrix(base_pose[0], base_pose[1], base_pose[2],
                                                base_pose[3], base_pose[4], base_pose[5])
    end_matrix = create_transformation_matrix(pose_data[0], pose_data[1], pose_data[2],
                                            pose_data[3], pose_data[4], pose_data[5])
    return np.dot(np.linalg.inv(begin_matrix), end_matrix)

def clamp(value, lower, upper):
    return max(lower, min(upper, value))

def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))

def rotation_vector_from_matrix(rotation):
    rotation = np.array(rotation, dtype=float)
    cos_angle = clamp((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(cos_angle)
    if angle < 1e-6:
        return np.zeros(3)
    axis = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ]) / (2.0 * math.sin(angle))
    return axis * angle

def matrix_from_rotation_vector(rotvec):
    rotvec = np.array(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-6:
        return np.eye(3)
    axis = rotvec / angle
    x, y, z = axis
    skew = np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * np.dot(skew, skew)

def get_float_array_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            rospy.logwarn("%s is not a valid numeric list: %s; using default", name, value)
            value = default
    try:
        return np.array(value, dtype=float)
    except (TypeError, ValueError):
        rospy.logwarn("%s cannot be converted to floats: %s; using default", name, value)
        return np.array(default, dtype=float)

def get_int_array_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            rospy.logwarn("%s is not a valid integer list: %s; using default", name, value)
            value = default
    try:
        return np.array(value, dtype=int)
    except (TypeError, ValueError):
        rospy.logwarn("%s cannot be converted to integers: %s; using default", name, value)
        return np.array(default, dtype=int)

class Arm_IK:
    def __init__(self):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        rospack = rospkg.RosPack()
        
        package_path = rospack.get_path('piper_description') 
        package_root = os.path.dirname(package_path)

        urdf_path = os.path.join(package_path, 'urdf', 'piper_description.urdf')
        
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=[package_root]
        )
        
        self.mixed_jointsToLockIDs = ["joint7",
                                      "joint8"
                                      ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0)
        self.second_matrix = create_transformation_matrix(0.13, 0.0, 0.0, 0, 0, 0)  #第六轴到末端夹爪坐标的变换矩阵
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame('ee',
                      self.reduced_robot.model.getJointId('joint6'),
                      pin.SE3(
                          # pin.Quaternion(1, 0, 0, 0),
                          pin.Quaternion(q[3], q[0], q[1], q[2]),
                          np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),  # -y
                      ),
                      pin.FrameType.OP_FRAME)
        )
        self.reduced_robot.data = self.reduced_robot.model.createData()
        joint6_ik_abs_limit = math.radians(float(rospy.get_param("~joint6_ik_abs_limit_deg", 180.0)))
        if joint6_ik_abs_limit > 0.0 and self.reduced_robot.model.nq >= 6:
            self.reduced_robot.model.lowerPositionLimit[5] = min(
                self.reduced_robot.model.lowerPositionLimit[5],
                -joint6_ik_abs_limit,
            )
            self.reduced_robot.model.upperPositionLimit[5] = max(
                self.reduced_robot.model.upperPositionLimit[5],
                joint6_ik_abs_limit,
            )

        self.geom_model = pin.buildGeomFromUrdf(
            self.robot.model,
            urdf_path,
            pin.GeometryType.COLLISION,
            package_dirs=[package_root],
        )
        for i in range(4, 9):
            for j in range(0, 3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))
        self.geometry_data = pin.GeometryData(self.geom_model)

        default_q_reference = [0.0, 0.35, -0.75, 0.0, 0.05, 0.0]
        q_reference = get_float_array_param("~joint_reference", default_q_reference)
        if q_reference.size != self.reduced_robot.model.nq:
            rospy.logwarn("joint_reference length is %s, expected %s; using default bent-elbow reference",
                          q_reference.size, self.reduced_robot.model.nq)
            q_reference = np.array(default_q_reference, dtype=float)
        self.q_reference = np.minimum(
            np.maximum(q_reference, self.reduced_robot.model.lowerPositionLimit),
            self.reduced_robot.model.upperPositionLimit,
        )
        self.init_data = self.q_reference.copy()
        self.history_data = self.q_reference.copy()
        self.filtered_q = self.q_reference.copy()
        self.ik_filter_alpha = float(rospy.get_param("~ik_filter_alpha", 0.35))
        self.ik_deadband = float(rospy.get_param("~ik_deadband", 0.006))
        self.ik_position_cost_weight = float(rospy.get_param("~ik_position_cost_weight", 180.0))
        self.ik_regularization_weight = float(rospy.get_param("~ik_regularization_weight", 0.03))
        self.ik_regularization_weights = get_float_array_param(
            "~ik_regularization_weights",
            [0.001, 0.001, 0.0004, 0.02, 0.03, 0.02],
        )
        if self.ik_regularization_weights.size != self.reduced_robot.model.nq:
            rospy.logwarn("ik_regularization_weights length is %s, expected %s; using default",
                          self.ik_regularization_weights.size, self.reduced_robot.model.nq)
            self.ik_regularization_weights = np.array([0.001, 0.001, 0.0004, 0.02, 0.03, 0.02], dtype=float)
        self.enable_collision_check = bool(rospy.get_param("~enable_collision_check", False))
        self.ik_max_solution_jump = math.radians(float(rospy.get_param("~ik_max_solution_jump_deg", 55.0)))
        self.ik_max_wrist_jump = math.radians(float(rospy.get_param("~ik_max_wrist_jump_deg", 45.0)))
        self.joint5_soft_abs_limit = math.radians(float(rospy.get_param("~joint5_soft_abs_limit_deg", 55.0)))

        self.enable_vis = rospy.get_param("~meshcat_vis", False)
        self.vis = None
        if self.enable_vis:
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True)
            self.vis.loadViewerModel("pinocchio")
            self.vis.displayFrames(True, frame_ids=[113, 114], axis_length=0.15, axis_width=5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            frame_viz_names = ['ee_target']
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 10
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        # self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf = self.opti.parameter(4, 4)
        self.param_q_ref = self.opti.parameter(self.reduced_robot.model.nq)

        # self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf))
        # self.regularization = casadi.sumsqr(self.var_q)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3] 
        ori_error = error_vec[3:]  
        weight_position = float(rospy.get_param("~ik_position_weight", 1.0))
        self.base_orientation_weight = float(rospy.get_param("~ik_orientation_weight", 0.3))
        self.param_orientation_weight = self.opti.parameter()
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(self.param_orientation_weight * ori_error)
        self.regularization = 0
        for i in range(self.reduced_robot.model.nq):
            self.regularization += self.ik_regularization_weights[i] * (self.var_q[i] - self.param_q_ref[i])**2
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        if self.joint5_soft_abs_limit > 0.0 and self.reduced_robot.model.nq >= 5:
            joint5_lower = max(self.reduced_robot.model.lowerPositionLimit[4], -self.joint5_soft_abs_limit)
            joint5_upper = min(self.reduced_robot.model.upperPositionLimit[4], self.joint5_soft_abs_limit)
            self.opti.subject_to(self.opti.bounded(joint5_lower, self.var_q[4], joint5_upper))
        # print("self.reduced_robot.model.lowerPositionLimit:", self.reduced_robot.model.lowerPositionLimit)
        # print("self.reduced_robot.model.upperPositionLimit:", self.reduced_robot.model.upperPositionLimit)
        self.opti.minimize(self.ik_position_cost_weight * self.totalcost + self.ik_regularization_weight * self.regularization)
        # self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization + 0.1 * self.smooth_cost) # for smooth

        opts = {
            'ipopt': {
                'print_level': 0,
                'max_iter': 50,
                'tol': 1e-4
            },
            'print_time': False
        }
        self.opti.solver("ipopt", opts)

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None, q_reference=None, orientation_weight=None):
        if target_pose is None or not np.all(np.isfinite(target_pose)):
            rospy.logwarn_throttle(1.0, "IK target contains invalid values; skipping command")
            return None, '', False
        gripper = np.array([gripper/2.0, -gripper/2.0])
        if motorstate is not None:
            motorstate = np.array(motorstate, dtype=float)
            motorstate = np.minimum(
                np.maximum(motorstate, self.reduced_robot.model.lowerPositionLimit),
                self.reduced_robot.model.upperPositionLimit,
            )
            self.init_data = motorstate
            self.filtered_q = motorstate.copy()
            self.history_data = motorstate.copy()
        self.opti.set_initial(self.var_q, self.init_data)

        if self.enable_vis:
            self.vis.viewer['ee_target'].set_transform(target_pose)

        self.opti.set_value(self.param_tf, target_pose)
        if q_reference is None:
            q_reference = self.q_reference
        q_reference = np.array(q_reference, dtype=float)
        if q_reference.size != self.reduced_robot.model.nq:
            q_reference = self.q_reference
        q_reference = np.minimum(
            np.maximum(q_reference, self.reduced_robot.model.lowerPositionLimit),
            self.reduced_robot.model.upperPositionLimit,
        )
        if orientation_weight is None:
            orientation_weight = self.base_orientation_weight
        self.opti.set_value(self.param_q_ref, q_reference)
        self.opti.set_value(self.param_orientation_weight, float(orientation_weight))
        # self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            # sol = self.opti.solve()
            sol = self.opti.solve_limited()
            raw_sol_q = self.opti.value(self.var_q)
            raw_delta = raw_sol_q - self.init_data
            max_raw_jump = float(np.max(np.abs(raw_delta)))
            max_wrist_jump = float(np.max(np.abs(raw_delta[3:6]))) if raw_delta.size >= 6 else 0.0
            if max_raw_jump > self.ik_max_solution_jump or max_wrist_jump > self.ik_max_wrist_jump:
                rospy.logwarn_throttle(
                    1.0,
                    "Rejecting IK branch jump: max=%.3f rad wrist=%.3f rad",
                    max_raw_jump,
                    max_wrist_jump,
                )
                return None, '', False
            if self.filtered_q is None:
                sol_q = raw_sol_q
            else:
                delta = raw_sol_q - self.filtered_q
                delta[np.abs(delta) < self.ik_deadband] = 0.0
                sol_q = self.filtered_q + self.ik_filter_alpha * delta
            self.filtered_q = sol_q

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                # print("max_diff:", max_diff)
                self.init_data = sol_q
                if max_diff > 30.0/180.0*3.1415:
                    rospy.logwarn_throttle(1.0, "Large IK joint jump %.3f rad", max_diff)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            if self.enable_vis:
                self.vis.display(sol_q)

            if motorV is not None:
                v = motorV * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            tau_ff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v,
                              np.zeros(self.reduced_robot.model.nv))

            is_collision = self.check_self_collision(sol_q, gripper) if self.enable_collision_check else False
            dist = self.get_dist(sol_q, target_pose[:3, 3])
            # print("dist:", dist)
            return sol_q, tau_ff, is_collision

        except Exception as e:
            rospy.logwarn_throttle(1.0, "IK solver failed: %s", e)
            # sol_q = self.opti.debug.value(self.var_q)   # return original value
            return None, '', False

    def check_self_collision(self, q, gripper=np.array([0, 0])):
        pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q, gripper], axis=0))
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        # print("collision:", collision)
        return collision

    def get_dist(self, q, xyz):
        # print("q:", q)
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        dist = math.sqrt(pow((xyz[0] - self.reduced_robot.data.oMi[6].translation[0]), 2) + pow((xyz[1] - self.reduced_robot.data.oMi[6].translation[1]), 2) + pow((xyz[2] - self.reduced_robot.data.oMi[6].translation[2]), 2))
        return dist

    def get_ee_matrix(self, q):
        q = np.array(q, dtype=float)
        data = self.reduced_robot.model.createData()
        pin.framesForwardKinematics(self.reduced_robot.model, data, q)
        if self.gripper_id >= len(data.oMf):
            rospy.logwarn_throttle(
                1.0,
                "ee frame id %s is outside oMf length %s; falling back to joint6 pose",
                self.gripper_id,
                len(data.oMf),
            )
            pin.forwardKinematics(self.reduced_robot.model, data, q)
            return data.oMi[self.reduced_robot.model.getJointId('joint6')].homogeneous.copy()
        return data.oMf[self.gripper_id].homogeneous.copy()

    def get_pose(self, q):
        index = 6
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(self.reduced_robot.data.oMi[index].translation[0], self.reduced_robot.data.oMi[index].translation[1], self.reduced_robot.data.oMi[index].translation[2],
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]),
                                                math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
                                                math.atan2(self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]))
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix_to_xyzrpy(end_pose)

class VR:
    def __init__(self):
        self.piper_control = PIPER()
        self.tools = MATHTOOLS()
        self.R_inverse_solution = Arm_IK()
        self.L_inverse_solution = Arm_IK()

        self.neutral_pose = np.array(rospy.get_param("~neutral_pose", [0.19, 0.0, 0.2, 0, 0, 0]), dtype=float)
        self.position_scale = float(rospy.get_param("~position_scale", 0.35))
        self.orientation_scale = float(rospy.get_param("~orientation_scale", 0.35))
        self.workspace_min = np.array(rospy.get_param("~workspace_min", [0.10, -0.30, 0.06]), dtype=float)
        self.workspace_max = np.array(rospy.get_param("~workspace_max", [0.42, 0.30, 0.42]), dtype=float)
        self.position_axis_map = get_int_array_param("~position_axis_map", [0, 1, 2])
        self.position_axis_sign = get_float_array_param("~position_axis_sign", [1.0, 1.0, 1.0])
        self.position_axis_scale = get_float_array_param("~position_axis_scale", [1.0, 2.8, 1.0])
        self.right_position_axis_sign = get_float_array_param("~right_position_axis_sign", [1.0, 1.0, 1.0])
        self.left_position_axis_sign = get_float_array_param("~left_position_axis_sign", [1.0, 1.0, 1.0])
        self.right_position_axis_scale = get_float_array_param("~right_position_axis_scale", [1.3, 0.7, 1.0])
        self.left_position_axis_scale = get_float_array_param("~left_position_axis_scale", [1.3, 0.7, 1.0])
        self.orientation_axis_map = get_int_array_param("~orientation_axis_map", [0, 1, 2])
        self.orientation_axis_sign = get_float_array_param("~orientation_axis_sign", [1.0, 1.0, 1.0])
        self.right_orientation_axis_sign = get_float_array_param("~right_orientation_axis_sign", [1.0, -1.0, 1.0])
        self.left_orientation_axis_sign = get_float_array_param("~left_orientation_axis_sign", [1.0, -1.0, 1.0])
        self.right_orientation_scale = float(rospy.get_param("~right_orientation_scale", self.orientation_scale))
        self.left_orientation_scale = float(rospy.get_param("~left_orientation_scale", 1.2))
        self.near_base_position_priority = bool(rospy.get_param("~near_base_position_priority", True))
        self.near_base_x_threshold = float(rospy.get_param("~near_base_x_threshold", 0.26))
        self.near_base_down_delta = float(rospy.get_param("~near_base_down_delta", -0.001))
        self.near_base_orientation_scale = float(rospy.get_param("~near_base_orientation_scale", 0.35))
        self.near_base_orientation_weight = float(rospy.get_param("~near_base_orientation_weight", 0.08))
        self.near_base_ref_blend = clamp(float(rospy.get_param("~near_base_ref_blend", 0.65)), 0.0, 1.0)
        self.near_base_joint_reference = get_float_array_param("~near_base_joint_reference", [0.0, 0.65, -1.25, 0.0, 0.05, 0.0])
        self.max_delta_position = float(rospy.get_param("~max_delta_position", 0.035))
        self.max_delta_rpy = math.radians(float(rospy.get_param("~max_delta_rpy_deg", 6.0)))
        if self.position_axis_map.size != 3 or any(axis not in (0, 1, 2) for axis in self.position_axis_map):
            rospy.logwarn("position_axis_map must be a permutation of [0, 1, 2]; using identity")
            self.position_axis_map = np.array([0, 1, 2], dtype=int)
        if self.position_axis_sign.size != 3:
            rospy.logwarn("position_axis_sign length must be 3; using all positive")
            self.position_axis_sign = np.array([1.0, 1.0, 1.0], dtype=float)
        if self.right_position_axis_sign.size != 3:
            rospy.logwarn("right_position_axis_sign length must be 3; using [1.0, 1.0, 1.0]")
            self.right_position_axis_sign = np.array([1.0, 1.0, 1.0], dtype=float)
        if self.left_position_axis_sign.size != 3:
            rospy.logwarn("left_position_axis_sign length must be 3; using [1.0, 1.0, 1.0]")
            self.left_position_axis_sign = np.array([1.0, 1.0, 1.0], dtype=float)
        if self.right_position_axis_scale.size != 3:
            rospy.logwarn("right_position_axis_scale length must be 3; using [1.3, 0.7, 1.0]")
            self.right_position_axis_scale = np.array([1.3, 0.7, 1.0], dtype=float)
        if self.left_position_axis_scale.size != 3:
            rospy.logwarn("left_position_axis_scale length must be 3; using [1.3, 0.7, 1.0]")
            self.left_position_axis_scale = np.array([1.3, 0.7, 1.0], dtype=float)
        if self.position_axis_scale.size != 3:
            rospy.logwarn("position_axis_scale length must be 3; using [1.0, 2.8, 1.0]")
            self.position_axis_scale = np.array([1.0, 2.8, 1.0], dtype=float)
        if self.orientation_axis_map.size != 3 or any(axis not in (0, 1, 2) for axis in self.orientation_axis_map):
            rospy.logwarn("orientation_axis_map must be a permutation of [0, 1, 2]; using identity")
            self.orientation_axis_map = np.array([0, 1, 2], dtype=int)
        if self.orientation_axis_sign.size != 3:
            rospy.logwarn("orientation_axis_sign length must be 3; using all positive")
            self.orientation_axis_sign = np.array([1.0, 1.0, 1.0], dtype=float)
        if self.right_orientation_axis_sign.size != 3:
            rospy.logwarn("right_orientation_axis_sign length must be 3; using [1.0, -1.0, 1.0]")
            self.right_orientation_axis_sign = np.array([1.0, -1.0, 1.0], dtype=float)
        if self.left_orientation_axis_sign.size != 3:
            rospy.logwarn("left_orientation_axis_sign length must be 3; using [1.0, -1.0, 1.0]")
            self.left_orientation_axis_sign = np.array([1.0, -1.0, 1.0], dtype=float)
        if self.near_base_joint_reference.size != 6:
            rospy.logwarn("near_base_joint_reference length must be 6; using bent-elbow default")
            self.near_base_joint_reference = np.array([0.0, 0.65, -1.25, 0.0, 0.05, 0.0], dtype=float)
        self.rpy_limit = float(rospy.get_param("~rpy_limit", 1.2))
        self.command_interval = rospy.Duration(1.0 / max(float(rospy.get_param("~command_hz", 12.0)), 1.0))
        self.enable_grip_threshold = float(rospy.get_param("~enable_grip_threshold", 0.25))
        self.reset_hold_sec = float(rospy.get_param("~reset_hold_sec", 0.0))
        self.debug_buttons = bool(rospy.get_param("~debug_buttons", False))
        self.debug_pose = bool(rospy.get_param("~debug_pose", False))
        self.control_mode = rospy.get_param("~control_mode", "delta_ik")
        self.enable_collision_check = bool(rospy.get_param("~enable_collision_check", False))
        self.collision_log_period = float(rospy.get_param("~collision_log_period", 2.0))
        self.last_right_command_time = rospy.Time(0)
        self.last_left_command_time = rospy.Time(0)
        self.prev_A = False
        self.prev_X = False
        self.right_reset_started = None
        self.left_reset_started = None
        self.right_reset_fired = False
        self.left_reset_fired = False
        self.last_right_buttons = {}
        self.last_left_buttons = {}
        self.last_button_log_time = rospy.Time(0)
        self.last_pose_log_time = rospy.Time(0)
        self.last_collision_log_time = rospy.Time(0)
        self.right_prev_pose = None
        self.left_prev_pose = None
        self.right_q_cmd = None
        self.left_q_cmd = None
        self.right_active = False
        self.left_active = False

        if bool(rospy.get_param("~auto_init_on_start", False)):
            self.piper_control.left_init_pose()
            self.piper_control.right_init_pose()
        
        # 这里可选为 WIFI连接 或 USB连接
        # oculus_reader = OculusReader(ip_address='10.12.11.14')    #  WIFI连接
        self.oculus_reader = OculusReader()                         #  USB连接
        
        # 延时0.5秒，确保 OculusReader 初始化完成   
        import time
        time.sleep(0.5)
        
        # 夹爪坐标系到到基坐标系的变换，仅用于 legacy_absolute 模式。
        self.base_RR = self.neutral_pose.tolist()
        self.base_LL = self.neutral_pose.tolist()

        # 订阅回调
        rospy.Subscriber('/right_handle_pose', PoseStamped, self.right_handle_pose_callback, queue_size=1)
        rospy.Subscriber('/left_handle_pose', PoseStamped, self.left_handle_pose_callback, queue_size=1)

    def _shape_pose_for_mobile_base(self, pose):
        pose = np.array(pose, dtype=float)
        shaped = self.neutral_pose.copy()
        delta = pose[:3] - self.neutral_pose[:3]
        mapped_delta = delta[self.position_axis_map] * self.position_axis_sign * self.position_axis_scale
        shaped[:3] = self.neutral_pose[:3] + mapped_delta * self.position_scale
        shaped[3:] = self.neutral_pose[3:] + (pose[3:] - self.neutral_pose[3:]) * self.orientation_scale
        shaped[:3] = np.minimum(np.maximum(shaped[:3], self.workspace_min), self.workspace_max)
        shaped[3:] = [clamp(v, -self.rpy_limit, self.rpy_limit) for v in shaped[3:]]
        return shaped.tolist()

    def _can_send(self, side):
        now = rospy.Time.now()
        if side == "right":
            if now - self.last_right_command_time < self.command_interval:
                return False
            self.last_right_command_time = now
            return True
        if now - self.last_left_command_time < self.command_interval:
            return False
        self.last_left_command_time = now
        return True

    @staticmethod
    def _button_pressed(buttons, name):
        if not isinstance(buttons, dict):
            return False
        return bool(buttons.get(name, False))

    @staticmethod
    def _axis_value(buttons, name):
        if not isinstance(buttons, dict):
            return 0.0
        value = buttons.get(name, 0.0)
        if isinstance(value, (list, tuple, np.ndarray)):
            if len(value) == 0:
                return 0.0
            value = value[0]
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _side_buttons(self, buttons, side):
        if side == "right":
            defaults = {"A": False, "B": False, "RThU": False, "RJ": False, "RG": False, "RTr": False}
            keys = ("A", "B", "RThU", "RJ", "RG", "RTr", "rightJS", "rightGrip", "rightTrig")
            self.last_right_buttons = defaults
        else:
            defaults = {"X": False, "Y": False, "LThU": False, "LJ": False, "LG": False, "LTr": False}
            keys = ("X", "Y", "LThU", "LJ", "LG", "LTr", "leftJS", "leftGrip", "leftTrig")
            self.last_left_buttons = defaults
        if not isinstance(buttons, dict):
            return defaults
        state = dict(defaults)
        for key in keys:
            if key in buttons:
                state[key] = buttons[key]
        if side == "right":
            self.last_right_buttons = state
        else:
            self.last_left_buttons = state
        return state

    def _right_teleop_enabled(self, buttons):
        return (
            self._button_pressed(buttons, "B")
            or self._button_pressed(buttons, "RG")
            or self._axis_value(buttons, "rightGrip") > self.enable_grip_threshold
        )

    def _left_teleop_enabled(self, buttons):
        return (
            self._button_pressed(buttons, "Y")
            or self._button_pressed(buttons, "LG")
            or self._axis_value(buttons, "leftGrip") > self.enable_grip_threshold
        )

    def _reset_ready(self, side, pressed):
        now = rospy.Time.now()
        if side == "right":
            if not pressed:
                self.right_reset_started = None
                self.right_reset_fired = False
                return False
            if self.reset_hold_sec <= 0.0:
                if self.right_reset_fired:
                    return False
                self.right_reset_fired = True
                return True
            if self.right_reset_started is None:
                self.right_reset_started = now
                return False
            if not self.right_reset_fired and (now - self.right_reset_started).to_sec() >= self.reset_hold_sec:
                self.right_reset_fired = True
                return True
            return False

        if not pressed:
            self.left_reset_started = None
            self.left_reset_fired = False
            return False
        if self.reset_hold_sec <= 0.0:
            if self.left_reset_fired:
                return False
            self.left_reset_fired = True
            return True
        if self.left_reset_started is None:
            self.left_reset_started = now
            return False
        if not self.left_reset_fired and (now - self.left_reset_started).to_sec() >= self.reset_hold_sec:
            self.left_reset_fired = True
            return True
        return False

    def _log_buttons(self, right_buttons, left_buttons):
        if not self.debug_buttons:
            return
        now = rospy.Time.now()
        if now - self.last_button_log_time < rospy.Duration(1.0):
            return
        self.last_button_log_time = now
        rospy.loginfo(
            "buttons right: A=%s B=%s RG=%s grip=%.2f | left: X=%s Y=%s LG=%s grip=%.2f",
            self._button_pressed(right_buttons, "A"),
            self._button_pressed(right_buttons, "B"),
            self._button_pressed(right_buttons, "RG"),
            self._axis_value(right_buttons, "rightGrip"),
            self._button_pressed(left_buttons, "X"),
            self._button_pressed(left_buttons, "Y"),
            self._button_pressed(left_buttons, "LG"),
            self._axis_value(left_buttons, "leftGrip"),
        )

    def _log_pose(self, side, pose_increment, shaped_pose):
        if not self.debug_pose:
            return
        now = rospy.Time.now()
        if now - self.last_pose_log_time < rospy.Duration(0.5):
            return
        self.last_pose_log_time = now
        rospy.loginfo(
            "%s pose inc xyz=(%.3f %.3f %.3f) rpy=(%.3f %.3f %.3f) target xyz=(%.3f %.3f %.3f) rpy=(%.3f %.3f %.3f)",
            side,
            pose_increment[0], pose_increment[1], pose_increment[2],
            pose_increment[3], pose_increment[4], pose_increment[5],
            shaped_pose[0], shaped_pose[1], shaped_pose[2],
            shaped_pose[3], shaped_pose[4], shaped_pose[5],
        )

    def _log_collision(self):
        if not self.enable_collision_check:
            return
        now = rospy.Time.now()
        if now - self.last_collision_log_time < rospy.Duration(self.collision_log_period):
            return
        self.last_collision_log_time = now
        rospy.logwarn("IK result reports self-collision")

    def _joint_seed(self, side, ik_solver):
        seed = self.piper_control.get_joint_seed(side, ik_solver.history_data.copy())
        if seed is None:
            seed = ik_solver.history_data.copy()
        return np.array(seed[:6], dtype=float)

    def _position_axis_sign_for_side(self, side):
        if side == "right":
            return self.position_axis_sign * self.right_position_axis_sign
        return self.position_axis_sign * self.left_position_axis_sign

    def _position_axis_scale_for_side(self, side):
        if side == "right":
            return self.position_axis_scale * self.right_position_axis_scale
        return self.position_axis_scale * self.left_position_axis_scale

    def _orientation_scale_for_side(self, side):
        if side == "right":
            return self.right_orientation_scale
        return self.left_orientation_scale

    def _orientation_axis_sign_for_side(self, side):
        if side == "right":
            return self.orientation_axis_sign * self.right_orientation_axis_sign
        return self.orientation_axis_sign * self.left_orientation_axis_sign

    def _target_from_increment(self, side, ik_solver, q_cmd, pose_delta_matrix):
        current_matrix = ik_solver.get_ee_matrix(q_cmd)
        current_pose = matrix_to_xyzrpy(current_matrix)
        delta_xyz = np.array(pose_delta_matrix[:3, 3], dtype=float)
        mapped_delta = delta_xyz[self.position_axis_map] * self._position_axis_sign_for_side(side) * self._position_axis_scale_for_side(side)
        mapped_delta = np.clip(mapped_delta, -self.max_delta_position, self.max_delta_position)

        target_xyz = np.array(current_pose[:3], dtype=float) + mapped_delta * self.position_scale
        position_priority = (
            self.near_base_position_priority
            and target_xyz[0] <= self.near_base_x_threshold
            and mapped_delta[2] < self.near_base_down_delta
        )
        delta_rotvec = rotation_vector_from_matrix(pose_delta_matrix[:3, :3])
        mapped_rotvec = delta_rotvec[self.orientation_axis_map] * self._orientation_axis_sign_for_side(side)
        mapped_rotvec = np.clip(mapped_rotvec * self._orientation_scale_for_side(side), -self.max_delta_rpy, self.max_delta_rpy)
        if position_priority:
            mapped_rotvec *= self.near_base_orientation_scale
        target_rotation = np.dot(current_matrix[:3, :3], matrix_from_rotation_vector(mapped_rotvec))
        target_matrix = np.eye(4)
        target_matrix[:3, :3] = target_rotation
        target_rpy = np.array(matrix_to_xyzrpy(target_matrix)[3:], dtype=float)
        target_xyz = np.minimum(np.maximum(target_xyz, self.workspace_min), self.workspace_max)
        target_rpy = np.array([clamp(v, -self.rpy_limit, self.rpy_limit) for v in target_rpy])
        context = {
            "position_priority": position_priority,
            "target_x": float(target_xyz[0]),
            "delta_z": float(mapped_delta[2]),
        }
        return np.concatenate([target_xyz, target_rpy]).tolist(), context

    def _near_base_reference(self, ik_solver):
        return (
            (1.0 - self.near_base_ref_blend) * ik_solver.q_reference
            + self.near_base_ref_blend * self.near_base_joint_reference
        )

    def _run_delta_ik(self, side, ik_solver, pose_data, gripper, enabled, publish_fn):
        if not enabled:
            if side == "right":
                self.right_active = False
                self.right_prev_pose = None
            else:
                self.left_active = False
                self.left_prev_pose = None
            return

        if side == "right":
            if not self.right_active:
                self.right_q_cmd = self._joint_seed("right", ik_solver)
                self.right_prev_pose = pose_data
                self.right_active = True
                return
            previous_pose = self.right_prev_pose
            q_cmd = self.right_q_cmd
        else:
            if not self.left_active:
                self.left_q_cmd = self._joint_seed("left", ik_solver)
                self.left_prev_pose = pose_data
                self.left_active = True
                return
            previous_pose = self.left_prev_pose
            q_cmd = self.left_q_cmd

        pose_delta_matrix = calc_pose_delta_matrix(previous_pose, pose_data)
        pose_increment = matrix_to_xyzrpy(np.dot(create_transformation_matrix(*self.neutral_pose), pose_delta_matrix))
        target_pose, ik_context = self._target_from_increment(side, ik_solver, q_cmd, pose_delta_matrix)
        self._log_pose(side, pose_increment, target_pose)

        q = quaternion_from_euler(target_pose[3], target_pose[4], target_pose[5])
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array(target_pose[:3]),
        )
        orientation_weight = None
        q_reference = None
        if ik_context["position_priority"]:
            orientation_weight = self.near_base_orientation_weight
            q_reference = self._near_base_reference(ik_solver)
            rospy.loginfo_throttle(
                1.0,
                "%s near-base position priority: x=%.3f dz=%.4f orientation_weight=%.3f",
                side,
                ik_context["target_x"],
                ik_context["delta_z"],
                orientation_weight,
            )
        sol_q, tau_ff, is_collision = ik_solver.ik_fun(
            target.homogeneous,
            0,
            motorstate=q_cmd,
            q_reference=q_reference,
            orientation_weight=orientation_weight,
        )
        if sol_q is not None:
            publish_fn(sol_q[0], sol_q[1], sol_q[2], sol_q[3], sol_q[4], sol_q[5], gripper)
            if side == "right":
                self.right_q_cmd = sol_q
            else:
                self.left_q_cmd = sol_q
        if side == "right":
            self.right_prev_pose = pose_data
        else:
            self.left_prev_pose = pose_data
        if is_collision:
            self._log_collision()
        
    def R_get_ik_solution(self, x,y,z,roll,pitch,yaw,gripper,b):
        if not b or not self._can_send("right"):
            return
        
        q = quaternion_from_euler(roll, pitch, yaw)
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([x, y, z]),
        )
        sol_q, tau_ff, is_collision = self.R_inverse_solution.ik_fun(target.homogeneous,0)
        # print("result:", sol_q)
        
        if sol_q is not None:
            self.piper_control.right_joint_control_piper(sol_q[0],sol_q[1],sol_q[2],sol_q[3],sol_q[4],sol_q[5],gripper)
        if is_collision :
            self._log_collision()

    def L_get_ik_solution(self, x,y,z,roll,pitch,yaw,gripper,b):
        if not b or not self._can_send("left"):
            return
        
        q = quaternion_from_euler(roll, pitch, yaw)
        target = pin.SE3(
            pin.Quaternion(q[3], q[0], q[1], q[2]),
            np.array([x, y, z]),
        )
        sol_q, tau_ff, is_collision = self.L_inverse_solution.ik_fun(target.homogeneous,0)
        # print("result:", sol_q)
        
        if sol_q is not None:
            self.piper_control.left_joint_control_piper(sol_q[0],sol_q[1],sol_q[2],sol_q[3],sol_q[4],sol_q[5],gripper)
        if is_collision :
            self._log_collision()

    def right_handle_pose_callback(self, msg):
        # print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        _, raw_buttons = self.oculus_reader.get_transformations_and_buttons()
        buttons = self._side_buttons(raw_buttons, "right")
        left_buttons = self._side_buttons(raw_buttons, "left")
        self._log_buttons(buttons, left_buttons)

        RR = [x,y,z,roll,pitch,yaw]
            
        a_pressed = self._button_pressed(buttons, 'A')
        if self._reset_ready("right", a_pressed):
            # 按下A键后，机械臂回到初始点位并且记录 右 坐标原点
            self.piper_control.right_init_pose()
            self.base_RR = [x,y,z,roll,pitch,yaw]
            self.right_active = False
            self.right_prev_pose = None
            self.right_q_cmd = None
        self.prev_A = a_pressed

        r_gripper_value = self._axis_value(buttons, 'rightTrig') * 0.07
        enabled = self._right_teleop_enabled(buttons)
        if self.control_mode == "delta_ik":
            self._run_delta_ik("right", self.R_inverse_solution, RR, r_gripper_value, enabled, self.piper_control.right_joint_control_piper)
        else:
            RR_incre = calc_pose_incre(self.base_RR,RR)
            RR_ = self._shape_pose_for_mobile_base(RR_incre)
            self._log_pose("right", RR_incre, RR_)
            # legacy_absolute: 按下B键后，开始遥操作
            self.R_get_ik_solution(RR_[0],RR_[1],RR_[2],RR_[3],RR_[4],RR_[5],r_gripper_value,enabled)


    def left_handle_pose_callback(self, msg):
        # print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        _, raw_buttons = self.oculus_reader.get_transformations_and_buttons()
        buttons = self._side_buttons(raw_buttons, "left")
        right_buttons = self._side_buttons(raw_buttons, "right")
        self._log_buttons(right_buttons, buttons)

        LL = [x,y,z,roll,pitch,yaw]
            
        x_pressed = self._button_pressed(buttons, 'X')
        if self._reset_ready("left", x_pressed):
            # 按下X键后，机械臂回到初始点位并且记录 左 坐标原点
            self.piper_control.left_init_pose()
            self.base_LL = [x,y,z,roll,pitch,yaw]
            self.left_active = False
            self.left_prev_pose = None
            self.left_q_cmd = None
        self.prev_X = x_pressed

        r_gripper_value = self._axis_value(buttons, 'leftTrig') * 0.07
        enabled = self._left_teleop_enabled(buttons)
        if self.control_mode == "delta_ik":
            self._run_delta_ik("left", self.L_inverse_solution, LL, r_gripper_value, enabled, self.piper_control.left_joint_control_piper)
        else:
            LL_incre = calc_pose_incre(self.base_LL,LL)
            RR_ = self._shape_pose_for_mobile_base(LL_incre)
            self._log_pose("left", LL_incre, RR_)
            # legacy_absolute: 按下Y键后，开始遥操作
            self.L_get_ik_solution(RR_[0],RR_[1],RR_[2],RR_[3],RR_[4],RR_[5],r_gripper_value,enabled)

if __name__ == '__main__':
    rospy.init_node('teleop_double_piper_node')
    vr = VR()
    rospy.spin()
    
