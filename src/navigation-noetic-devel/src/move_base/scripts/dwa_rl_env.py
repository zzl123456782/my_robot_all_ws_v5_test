#!/usr/bin/env python3
import rospy
import numpy as np
import time
from collections import deque
import math
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class DWARLTrainingEnv:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('dwa_rl_training_env', anonymous=True)
        
        # 状态和动作空间维度
        self.state_dim = 28  # 24激光 + 2目标 + 2速度
        self.action_dim = 5  # 5个DWA参数调整量
        
        # 机器人状态
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.current_vel = Twist()
        self.laser_data = None
        self.goal_position = [3.0, 0.0]  # 默认目标：前方3米
        self.start_position = [0.0, 0.0, 0.0]
        
        # 训练参数
        self.max_steps_per_episode = 500
        self.current_step = 0
        self.episode_count = 0
        self.success_count = 0
        self.total_reward = 0
        
        # 目标生成模式
        self.goal_mode = "curriculum"  # 可选: "random", "curriculum", "waypoints", "adaptive"
        self.difficulty_level = 1  # 难度等级1-4
        
        # 预定义路径点
        self.waypoints = [
            [2.0, 0.0],    # 前方2米
            [2.0, 1.5],    # 右前方
            [1.5, 2.0],    # 右侧偏前
            [0.0, 2.0],    # 正右侧
            [-1.5, 2.0],   # 右侧偏后
            [-2.0, 1.5],   # 右后方
            [2.0, -1.5],   # 左前方
            [1.5, -2.0],   # 左侧偏前
            [0.0, -2.0],   # 正左侧
            [-1.5, -2.0],  # 左侧偏后
            [-2.0, -1.5]   # 左后方
        ]
        
        # 历史记录
        self.success_history = deque(maxlen=20)
        self.reward_history = deque(maxlen=100)
        
        # 订阅器和发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 重置仿真的服务
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # 等待话题
        rospy.sleep(1)
        rospy.loginfo("DWA RL训练环境初始化完成")
    
    def odom_callback(self, msg):
        """里程计回调，获取机器人位姿和速度"""
        # 位置
        pose = msg.pose.pose
        self.robot_pose[0] = pose.position.x
        self.robot_pose[1] = pose.position.y
        
        # 姿态（四元数转欧拉角）
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw
        
        # 速度
        self.current_vel.linear.x = msg.twist.twist.linear.x
        self.current_vel.angular.z = msg.twist.twist.angular.z
    
    def scan_callback(self, msg):
        """激光雷达回调"""
        self.laser_data = msg
    
    def get_state(self):
        """获取当前状态向量"""
        state = []
        
        # 1. 处理激光数据 (24维)
        if self.laser_data is not None:
            ranges = np.array(self.laser_data.ranges)
            ranges = np.clip(ranges, 0.1, 3.5)
            ranges = np.nan_to_num(ranges, nan=3.5)
            
            if len(ranges) >= 24:
                indices = np.linspace(0, len(ranges)-1, 24, dtype=int)
                state.extend(ranges[indices])
            else:
                state.extend([3.5] * 24)
        else:
            state.extend([3.5] * 24)
        
        # 2. 目标相对位置 (2维)
        dx = self.goal_position[0] - self.robot_pose[0]
        dy = self.goal_position[1] - self.robot_pose[1]
        
        # 距离
        distance = np.sqrt(dx**2 + dy**2)
        
        # 相对角度（全局角度 - 机器人朝向）
        goal_angle_global = np.arctan2(dy, dx)
        robot_angle = self.robot_pose[2]
        relative_angle = goal_angle_global - robot_angle
        
        # 归一化到[-pi, pi]
        relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
        
        state.append(distance)
        state.append(relative_angle)
        
        # 3. 当前速度 (2维)
        state.append(self.current_vel.linear.x)
        state.append(self.current_vel.angular.z)
        
        return np.array(state, dtype=np.float32)
    
    def generate_goal_position(self, episode=None):
        """根据策略生成目标点"""
        if episode is None:
            episode = self.episode_count
            
        if self.goal_mode == "random":
            return self._generate_random_goal()
        elif self.goal_mode == "curriculum":
            return self._generate_curriculum_goal(episode)
        elif self.goal_mode == "waypoints":
            return self._generate_waypoint_goal(episode)
        elif self.goal_mode == "adaptive":
            return self._generate_adaptive_goal()
        else:
            return [3.0, 0.0]  # 默认
    
    def _generate_random_goal(self):
        """完全随机目标"""
        # 在半径为1-5米的圆环内随机
        r = np.random.uniform(1.5, 4.0)
        theta = np.random.uniform(-np.pi, np.pi)
        
        goal_x = r * np.cos(theta)
        goal_y = r * np.sin(theta)
        
        return [goal_x, goal_y]
    
    def _generate_curriculum_goal(self, episode):
        """课程学习：从易到难"""
        if episode < 100:  # 阶段1：简单直行
            r = np.random.uniform(1.5, 2.5)
            theta = np.random.uniform(-0.3, 0.3)  # 小角度偏移
        elif episode < 300:  # 阶段2：中等转弯
            r = np.random.uniform(2.0, 3.5)
            theta = np.random.uniform(-np.pi/2, np.pi/2)  # ±90度
        elif episode < 600:  # 阶段3：复杂路径
            r = np.random.uniform(2.5, 4.0)
            theta = np.random.uniform(-np.pi*0.8, np.pi*0.8)  # 几乎全角度
        else:  # 阶段4：挑战性目标
            if np.random.random() < 0.3:
                # 近距离大角度
                r = np.random.uniform(1.0, 2.0)
                theta = np.random.choice([-np.pi*0.8, np.pi*0.8])
            else:
                # 远距离任意角度
                r = np.random.uniform(3.0, 5.0)
                theta = np.random.uniform(-np.pi, np.pi)
        
        goal_x = r * np.cos(theta)
        goal_y = r * np.sin(theta)
        return [goal_x, goal_y]
    
    def _generate_waypoint_goal(self, episode):
        """使用预定义路径点"""
        idx = episode % len(self.waypoints)
        return self.waypoints[idx]
    
    def _generate_adaptive_goal(self):
        """自适应难度目标"""
        if len(self.success_history) == 0:
            success_rate = 0.0
        else:
            success_rate = np.mean(self.success_history)
        
        # 根据成功率调整难度
        if success_rate > 0.8 and self.difficulty_level < 4:
            self.difficulty_level += 1
            rospy.loginfo(f"增加难度到等级 {self.difficulty_level}")
        elif success_rate < 0.4 and self.difficulty_level > 1:
            self.difficulty_level -= 1
            rospy.loginfo(f"降低难度到等级 {self.difficulty_level}")
        
        # 根据难度等级生成目标
        if self.difficulty_level == 1:
            r = np.random.uniform(1.5, 2.5)
            theta = np.random.uniform(-0.3, 0.3)
        elif self.difficulty_level == 2:
            r = np.random.uniform(2.0, 3.5)
            theta = np.random.uniform(-np.pi/3, np.pi/3)  # ±60度
        elif self.difficulty_level == 3:
            r = np.random.uniform(2.5, 4.0)
            theta = np.random.uniform(-np.pi/2, np.pi/2)  # ±90度
        else:  # 等级4
            r = np.random.uniform(3.0, 5.0)
            theta = np.random.uniform(-np.pi*0.9, np.pi*0.9)  # ±162度
        
        goal_x = r * np.cos(theta)
        goal_y = r * np.sin(theta)
        return [goal_x, goal_y]
    
    def publish_goal_marker(self, position, duration=0.1):
        """在Rviz中可视化目标点"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(duration)
        
        self.goal_pub.publish(marker)
    
    def reset(self, test_mode=False):
        """重置环境"""
        self.episode_count += 1
        self.current_step = 0
        self.total_reward = 0
        
        # 重置Gazebo仿真
        try:
            self.reset_simulation()
            rospy.sleep(0.5)  # 等待重置完成
        except:
            rospy.logwarn("无法重置仿真，继续...")
        
        # 重置机器人位置
        self.robot_pose = [0.0, 0.0, 0.0]
        self.start_position = [0.0, 0.0, 0.0]
        
        # 生成新目标点
        if test_mode:
            # 测试模式：固定目标
            self.goal_position = [3.0, 0.0]
        else:
            # 训练模式：根据策略生成
            self.goal_position = self.generate_goal_position()
        
        # 可视化目标点
        self.publish_goal_marker(self.goal_position, duration=30)
        
        # 等待数据稳定
        rospy.sleep(0.5)
        
        # 获取初始状态
        initial_state = self.get_state()
        
        rospy.loginfo(f"Episode {self.episode_count}: 目标点 ({self.goal_position[0]:.2f}, {self.goal_position[1]:.2f})")
        
        return initial_state
    
    def step(self, action):
        """执行一步动作"""
        self.current_step += 1
        
        # 保存之前状态用于奖励计算
        previous_state = self.get_state().copy()
        previous_distance = self._calculate_distance_to_goal()
        
        # 1. 更新DWA参数（这里action是参数的调整量）
        self._update_dwa_params(action)
        
        # 2. 基于当前状态计算最优速度
        cmd_vel = self._compute_velocity_from_state(previous_state)
        
        # 3. 发布控制命令
        self.cmd_pub.publish(cmd_vel)
        
        # 4. 等待一个时间步
        rospy.sleep(0.1)  # 10Hz控制频率
        
        # 5. 获取新状态
        new_state = self.get_state()
        new_distance = self._calculate_distance_to_goal()
        
        # 6. 计算奖励
        reward, done, success = self._calculate_reward(
            previous_state, new_state, 
            previous_distance, new_distance, 
            cmd_vel
        )
        
        self.total_reward += reward
        
        # 7. 检查终止条件
        if self.current_step >= self.max_steps_per_episode:
            done = True
            rospy.loginfo(f"达到最大步数: {self.max_steps_per_episode}")
        
        # 8. 记录成功率
        if success:
            self.success_count += 1
            self.success_history.append(1)
        elif done and not success:
            self.success_history.append(0)
        
        # 记录奖励历史
        self.reward_history.append(reward)
        
        return new_state, reward, done, {"success": success, "steps": self.current_step}
    
    def _update_dwa_params(self, action):
        """更新DWA参数（简化版本）"""
        # action是[-1, 1]范围的调整量
        # 在实际实现中，这里会更新DWA控制器的参数
        # 现在只是记录参数变化
        # self.last_action = action

        self.dwa_params = {
        'max_vel_x': 0.1 + (action[0] + 1) * 0.45,  # 0.1-1.0
        'min_vel_x': -0.5 + (action[1] + 1) * 0.25,  # -0.5-0.0
        'max_vel_theta': 0.5 + (action[2] + 1) * 0.75,  # 0.5-2.0
        'acc_lim_x': 0.5 + (action[3] + 1) * 0.75,  # 0.5-2.0
        'path_distance_bias': 0.1 + (action[4] + 1) * 2.45  # 0.1-5.0
    }
        
    def _compute_velocity_from_state(self, state):
        """基于状态计算控制速度（简化的DWA）"""
        cmd_vel = Twist()
        
        # 提取状态信息
        distances = state[:24]  # 24个激光距离
        goal_distance = state[24]
        goal_angle = state[25]
        current_vx = state[26]
        current_wz = state[27]
        
        # 基本避障逻辑
        min_distance = np.min(distances[8:16])  # 前方90度范围
        
        if min_distance < 0.3:  # 太近，需要转向
            # 找到较安全的方向
            left_dist = np.mean(distances[:8])  # 左侧
            right_dist = np.mean(distances[16:])  # 右侧
            
            if left_dist > right_dist:
                cmd_vel.angular.z = 0.5  # 左转
            else:
                cmd_vel.angular.z = -0.5  # 右转
            cmd_vel.linear.x = 0.1
        else:
            # 朝向目标移动
            if abs(goal_angle) > 0.5:  # 角度较大，先转向
                cmd_vel.angular.z = np.clip(goal_angle * 0.5, -1.0, 1.0)
                cmd_vel.linear.x = 0.1
            else:  # 基本朝向目标，前进
                cmd_vel.linear.x = np.clip(goal_distance * 0.3, 0.1, 0.3)
                cmd_vel.angular.z = np.clip(goal_angle * 0.8, -0.5, 0.5)
        
        return cmd_vel
    
    def _calculate_distance_to_goal(self):
        """计算到目标的距离"""
        dx = self.goal_position[0] - self.robot_pose[0]
        dy = self.goal_position[1] - self.robot_pose[1]
        return np.sqrt(dx**2 + dy**2)
    
    def _calculate_reward(self, prev_state, new_state, prev_dist, new_dist, cmd_vel):
        """计算奖励"""
        reward = 0
        done = False
        success = False
        
        # 1. 到达目标（主要奖励）
        if new_dist < 0.3:  # 到达目标阈值
            reward += 100
            done = True
            success = True
            rospy.loginfo(f"到达目标！距离: {new_dist:.3f}m")
            return reward, done, success
        
        # 2. 碰撞检测
        min_distance = np.min(new_state[:24])
        if min_distance < 0.15:  # 碰撞
            reward -= 50
            done = True
            rospy.loginfo(f"发生碰撞！最近距离: {min_distance:.3f}m")
            return reward, done, success
        
        # 3. 距离奖励：更接近目标
        distance_reward = (prev_dist - new_dist) * 10
        reward += distance_reward
        
        # 4. 效率奖励：鼓励前进
        if new_state[26] > 0.05:  # 有正向速度
            reward += new_state[26] * 2
        
        # 5. 角度奖励：朝向目标
        goal_angle = new_state[25]
        angle_reward = -abs(goal_angle) * 0.5
        reward += angle_reward
        
        # 6. 平滑性惩罚：避免剧烈转向
        angular_change = abs(cmd_vel.angular.z - prev_state[27])
        reward -= angular_change * 0.1
        
        # 7. 步数惩罚：鼓励快速到达
        reward -= 0.01
        
        return reward, done, success
    
    def get_training_info(self):
        """获取训练统计信息"""
        if len(self.success_history) > 0:
            success_rate = np.mean(self.success_history) * 100
        else:
            success_rate = 0
        
        if len(self.reward_history) > 0:
            avg_reward = np.mean(self.reward_history)
        else:
            avg_reward = 0
        
        return {
            "episode": self.episode_count,
            "success_rate": success_rate,
            "avg_reward": avg_reward,
            "total_success": self.success_count,
            "difficulty_level": self.difficulty_level
        }