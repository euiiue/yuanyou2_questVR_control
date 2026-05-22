#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import JointState
# from piper_msgs.msg import PosCmd
from std_msgs.msg import Header

class PIPER:
    def __init__(self):
        
        # 发布控制piper机械臂话题
        # self.pub_descartes = rospy.Publisher('pos_cmd', PosCmd, queue_size=10)
        self.pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.left_pub_joint = rospy.Publisher('/left_joint_states', JointState, queue_size=1)
        self.right_pub_joint = rospy.Publisher('/right_joint_states', JointState, queue_size=1)
        # self.descartes_msgs = PosCmd()
        
        # self.rate = rospy.Rate(80) # 10hz
        self.target_joint_state = rospy.get_param('~target_joint_state', default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.command_interval = rospy.Duration(1.0 / max(float(rospy.get_param('~joint_command_hz', 12.0)), 1.0))
        self.max_joint_step = math.radians(float(rospy.get_param('~max_joint_step_deg', 3.0)))
        self.command_deadband = math.radians(float(rospy.get_param('~command_deadband_deg', 0.35)))
        self.max_gripper_step = float(rospy.get_param('~max_gripper_step', 0.004))
        self.gripper_deadband = float(rospy.get_param('~gripper_deadband', 0.001))
        self.init_duration = float(rospy.get_param('~init_duration', 4.0))
        self.init_rate = float(rospy.get_param('~init_rate', 20.0))
        self.log_joint_commands = bool(rospy.get_param('~log_joint_commands', False))
        self.debug_joint3 = bool(rospy.get_param('~debug_joint3', False))
        self.last_feedback_log_time = {
            'single': rospy.Time(0),
            'left': rospy.Time(0),
            'right': rospy.Time(0),
        }
        self.last_command_time = {
            'single': rospy.Time(0),
            'left': rospy.Time(0),
            'right': rospy.Time(0),
        }
        self.last_positions = {
            'single': None,
            'left': None,
            'right': None,
        }
        self.feedback_positions = {
            'single': None,
            'left': None,
            'right': None,
        }
        
        # 订阅joint_states_single话题获取当前关节位置
        rospy.Subscriber(f'joint_states_single', JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber('/puppet/joint_left', JointState, self.left_joint_states_callback, queue_size=1)
        rospy.Subscriber('/puppet/joint_right', JointState, self.right_joint_states_callback, queue_size=1)
        
        # 存储当前关节位置
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 标记是否已获取到当前关节位置
        self.joint_positions_received = False

    def _make_joint_state(self, positions):
        joint_states_msgs = JointState()
        joint_states_msgs.header = Header()
        joint_states_msgs.header.stamp = rospy.Time.now()
        joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
        joint_states_msgs.position = list(positions)
        return joint_states_msgs

    def _rate_limited(self, side):
        now = rospy.Time.now()
        if now - self.last_command_time[side] < self.command_interval:
            return True
        self.last_command_time[side] = now
        return False

    def _limit_step(self, side, positions):
        positions = list(positions)
        previous = self.last_positions[side]
        if previous is None:
            previous = self.feedback_positions.get(side)
            if previous is None:
                return positions

        limited = []
        for i, (target, last) in enumerate(zip(positions, previous)):
            max_step = self.max_gripper_step if i == 6 else self.max_joint_step
            delta = max(-max_step, min(max_step, target - last))
            limited.append(last + delta)
        return limited

    def _within_deadband(self, previous, positions):
        if previous is None:
            return False
        for i, (last, current) in enumerate(zip(previous, positions)):
            deadband = self.gripper_deadband if i == 6 else self.command_deadband
            if abs(current - last) >= deadband:
                return False
        return True

    def _publish_joint(self, publisher, side, positions, force=False):
        if not force and self._rate_limited(side):
            return
        previous = self.last_positions[side]
        positions = self._limit_step(side, positions)
        if not force and self._within_deadband(previous, positions):
            return
        self.last_positions[side] = positions
        publisher.publish(self._make_joint_state(positions))
        if self.log_joint_commands:
            rospy.loginfo("%s joint command: %s", side, ["%.3f" % p for p in positions])
        if self.debug_joint3:
            rospy.loginfo("%s joint3 command %.4f rad", side, positions[2])

    def _feedback_callback(self, side, msg):
        if len(msg.position) >= 7:
            positions = list(msg.position[:7])
            self.feedback_positions[side] = positions
            if self.last_positions[side] is None:
                self.last_positions[side] = positions
            if self.debug_joint3:
                now = rospy.Time.now()
                if now - self.last_feedback_log_time[side] >= rospy.Duration(1.0):
                    self.last_feedback_log_time[side] = now
                    rospy.loginfo("%s joint3 feedback %.4f rad", side, positions[2])

    def get_joint_seed(self, side, default=None):
        positions = self.feedback_positions.get(side) or self.last_positions.get(side)
        if positions is None:
            return default
        return list(positions[:6])
        
    # def init_pose(self):
    #     joint_states_msgs = JointState()
    #     joint_states_msgs.header = Header()
    #     joint_states_msgs.header.stamp = rospy.Time.now()
    #     joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
    #     joint_states_msgs.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     self.pub_joint.publish(joint_states_msgs)
    #     # self.rate.sleep()
    #     print("send joint control piper command")
    
    def joint_states_callback(self, msg):
        """
        处理从joint_states_single话题接收到的关节状态数据
        """
        # 确保消息中包含关节位置数据
        if len(msg.position) >= 7:
            # 更新当前关节位置
            self.current_joint_positions = list(msg.position[:7])
            # 标记已接收到关节位置数据
            self.joint_positions_received = True
            self.feedback_positions['single'] = self.current_joint_positions
            # rospy.logdebug(f"接收到当前关节位置: {self.current_joint_positions}")

    def left_joint_states_callback(self, msg):
        self._feedback_callback('left', msg)

    def right_joint_states_callback(self, msg):
        self._feedback_callback('right', msg)

    def _smooth_init_pose(self, publisher, side):
        target_joint_state = list(self.target_joint_state)
        current_positions = self.feedback_positions.get(side) or self.last_positions.get(side)
        if current_positions is None:
            current_positions = target_joint_state
        else:
            current_positions = list(current_positions[:7])

        steps = max(1, int(self.init_duration * self.init_rate))
        rate_obj = rospy.Rate(self.init_rate)
        rospy.loginfo("Moving %s to init pose over %.1fs", side, self.init_duration)

        saved_step = self.max_joint_step
        try:
            self.max_joint_step = max(saved_step, math.radians(180.0) / steps)
            for step in range(1, steps + 1):
                alpha = float(step) / float(steps)
                interpolated = [
                    current + (target - current) * alpha
                    for current, target in zip(current_positions, target_joint_state)
                ]
                self._publish_joint(publisher, side, interpolated, force=True)
                rate_obj.sleep()
        finally:
            self.max_joint_step = saved_step
            
    # 使用线性插值实现平滑过渡到初始位置
    def init_pose(self):
        # 目标关节位置
        target_joint_state = self.target_joint_state
        
        # 获取当前关节位置
        # 如果已经接收到关节位置数据，使用实际的当前位置
        # 否则会一步调整到位
        if self.joint_positions_received:
            current_positions = self.current_joint_positions
            rospy.loginfo(f"使用实际的当前关节位置: {current_positions}")
            
            # 设置过渡时间和控制频率
            duration = self.init_duration
            rate = self.init_rate
            
            # 计算总步数
            steps = int(duration * rate)
            
            # 计算每一步的增量
            increments = [(target - current) / steps for current, target in zip(current_positions, target_joint_state)]
            
            # 创建ROS的Rate对象控制循环频率
            rate_obj = rospy.Rate(rate)
            
            # 记录开始时间（用于日志）
            start_time = rospy.Time.now()
            
            # 逐步移动到目标位置k
            for step in range(steps + 1):
                # 计算当前步骤的位置
                interpolated_positions = [current + increment * step for current, increment in zip(current_positions, increments)]
                
                self._publish_joint(self.pub_joint, 'single', interpolated_positions, force=True)
                
                # 按照指定频率控制循环
                rate_obj.sleep()
            
            # 确保最后一帧是精确的目标位置
            self._publish_joint(self.pub_joint, 'single', target_joint_state, force=True)
            
            # 计算实际用时
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            # print(f"平滑移动到初始位置完成，用时: {elapsed_time:.2f}秒")
            
        else:
            start_time = rospy.Time.now()  # 获取当前时间
            rate_obj = rospy.Rate(self.init_rate)
            while (rospy.Time.now() - start_time).to_sec() < self.init_duration:
                self._publish_joint(self.pub_joint, 'single', target_joint_state, force=True)
                rate_obj.sleep()
            # print("send joint control piper command for 2 seconds")
            # 使用默认的非零位置作为起始点
            # current_positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # rospy.loginfo("未接收到当前关节位置，使用默认初始位置")
            
        
        
    def left_init_pose(self):
        self._smooth_init_pose(self.left_pub_joint, 'left')
        
    def right_init_pose(self):
        self._smooth_init_pose(self.right_pub_joint, 'right')
        
    def descartes_control_piper(self,x,y,z,roll,pitch,yaw,gripper):
        self.descartes_msgs.x = x
        self.descartes_msgs.y = y
        self.descartes_msgs.z = z
        self.descartes_msgs.roll = roll
        self.descartes_msgs.pitch = pitch
        self.descartes_msgs.yaw = yaw
        self.descartes_msgs.gripper = gripper
        self.pub_descartes.publish(self.descartes_msgs)
        # print("send descartes control piper command")
    
    def joint_control_piper(self,j1,j2,j3,j4,j5,j6,gripper):
        self._publish_joint(self.pub_joint, 'single', [j1, j2, j3, j4, j5, j6, gripper])
    
    def left_joint_control_piper(self,j1,j2,j3,j4,j5,j6,gripper):
        self._publish_joint(self.left_pub_joint, 'left', [j1, j2, j3, j4, j5, j6, gripper])
        
    
    def right_joint_control_piper(self,j1,j2,j3,j4,j5,j6,gripper):
        self._publish_joint(self.right_pub_joint, 'right', [j1, j2, j3, j4, j5, j6, gripper])
    
    
     
# test code
# if __name__ == '__main__':
    # piper = PIPER() 
    # rospy.init_node('control_piper_node', anonymous=True)
    # piper.control_piper(0.0,0.0,0.0,0.0,0.0,0.0,0.05)
    # piper.init_pose()
    # 保持节点运行并监听外部程序的调用
    # rospy.spin()
