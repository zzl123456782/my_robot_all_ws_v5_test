#!/usr/bin/env python3
"""
DWA RL训练 - 修复版本
修复了缺失属性和TF问题
"""

import rospy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import math
import time
import os
import sys
import signal
from collections import deque
import random
from datetime import datetime
import csv
import json
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionGoal
from visualization_msgs.msg import Marker
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty

# 初始化ROS节点
rospy.init_node('dwa_rl_trainer', anonymous=True, log_level=rospy.INFO)

# ==================== 工具函数 ====================
def setup_tf_transforms():
    """设置必要的TF转换"""
    rospy.loginfo("设置TF转换...")
    rospy.set_param('/move_base/global_costmap/global_frame', 'map')
    rospy.set_param('/move_base/global_costmap/robot_base_frame', 'base_footprint')
    rospy.set_param('/move_base/local_costmap/global_frame', 'odom')
    rospy.set_param('/move_base/local_costmap/robot_base_frame', 'base_footprint')
    rospy.loginfo("TF参数已设置")

# ==================== 强化学习智能体 ====================
class ActorNetwork(nn.Module):
    """Actor网络"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim//2)
        self.fc4 = nn.Linear(hidden_dim//2, action_dim)
        
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = torch.tanh(self.fc4(x))
        return x

class CriticNetwork(nn.Module):
    """Critic网络"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()
        # Q1网络
        self.fc1 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, 1)
        
        # Q2网络
        self.fc4 = nn.Linear(state_dim + action_dim, hidden_dim)
        self.fc5 = nn.Linear(hidden_dim, hidden_dim)
        self.fc6 = nn.Linear(hidden_dim, 1)
    
    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        
        # Q1
        q1 = F.relu(self.fc1(x))
        q1 = F.relu(self.fc2(q1))
        q1 = self.fc3(q1)
        
        # Q2
        q2 = F.relu(self.fc4(x))
        q2 = F.relu(self.fc5(q2))
        q2 = self.fc6(q2)
        
        return q1, q2
    
    def Q1(self, state, action):
        """返回Q1值"""
        x = torch.cat([state, action], dim=1)
        q1 = F.relu(self.fc1(x))
        q1 = F.relu(self.fc2(q1))
        q1 = self.fc3(q1)
        return q1

class DWA_RL_Agent:
    """DWA RL智能体"""
    
    def __init__(self, state_dim, action_dim, device='cpu'):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = torch.device(device)
        
        rospy.loginfo(f"初始化RL智能体: 状态维度={state_dim}, 动作维度={action_dim}, 设备={device}")
        
        # Actor网络
        self.actor = ActorNetwork(state_dim, action_dim).to(self.device)
        self.actor_target = ActorNetwork(state_dim, action_dim).to(self.device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=0.001)
        
        # Critic网络
        self.critic = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_target = CriticNetwork(state_dim, action_dim).to(self.device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=0.002)
        
        # 经验回放
        self.memory = deque(maxlen=100000)
        
        # 训练参数
        self.gamma = 0.99
        self.tau = 0.005
        self.batch_size = 256
        self.noise_scale = 1.0
        self.noise_decay = 0.995
        self.noise_min = 0.1
        self.policy_noise = 0.2
        self.noise_clip = 0.5
        self.policy_freq = 2
        
        # 训练记录
        self.total_steps = 0
        self.actor_losses = []
        self.critic_losses = []
        
        rospy.loginfo("RL智能体初始化完成")
    
    def select_action(self, state, add_noise=True):
        """选择动作"""
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            action = self.actor(state_tensor).cpu().numpy()[0]
        
        if add_noise:
            noise = np.random.normal(0, self.noise_scale, self.action_dim)
            noise = np.clip(noise, -self.noise_clip, self.noise_clip)
            action = action + noise
        
        return np.clip(action, -1.0, 1.0)
    
    def remember(self, state, action, reward, next_state, done):
        """存储经验"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train(self):
        """训练网络"""
        if len(self.memory) < self.batch_size:
            return None, None
        
        # 采样批次
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        # 转换为张量
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.FloatTensor(np.array(actions)).to(self.device)
        rewards = torch.FloatTensor(np.array(rewards)).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(np.array(dones)).unsqueeze(1).to(self.device)
        
        with torch.no_grad():
            # 目标策略平滑
            noise = (torch.randn_like(actions) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
            next_actions = (self.actor_target(next_states) + noise).clamp(-1.0, 1.0)
            
            # 计算目标Q值
            target_q1, target_q2 = self.critic_target(next_states, next_actions)
            target_q = torch.min(target_q1, target_q2)
            target_q = rewards + (1 - dones) * self.gamma * target_q
        
        # 计算当前Q值
        current_q1, current_q2 = self.critic(states, actions)
        
        # Critic损失
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)
        
        # 优化Critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # Actor损失
        if self.total_steps % self.policy_freq == 0:
            actor_loss = -self.critic.Q1(states, self.actor(states)).mean()
            
            # 优化Actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
            self.actor_optimizer.step()
            
            # 软更新目标网络
            for target_param, param in zip(self.actor_target.parameters(), self.actor.parameters()):
                target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)
            
            for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
                target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)
            
            self.actor_losses.append(actor_loss.item())
        
        # 衰减噪声
        self.noise_scale = max(self.noise_min, self.noise_scale * self.noise_decay)
        
        # 记录
        self.total_steps += 1
        critic_loss_value = critic_loss.item()
        self.critic_losses.append(critic_loss_value)
        
        actor_loss_value = actor_loss.item() if actor_loss is not None else None
        
        return actor_loss_value, critic_loss_value

# ==================== 训练环境 ====================
class DWATrainingEnv:
    """DWA训练环境"""
    
    def __init__(self):
        rospy.loginfo("初始化DWA训练环境...")
        
        # 设置TF参数
        setup_tf_transforms()
        
        # ROS发布器和订阅器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        # Action客户端
        rospy.loginfo("等待move_base服务器...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0))
        rospy.loginfo("move_base服务器已连接")
        
        # 机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        
        # 激光数据
        self.laser_ranges = None
        self.laser_angles = None
        
        # 初始化激光数据
        self.laser_ranges = np.ones(360, dtype=np.float32) * 3.5
        self.laser_angles = np.linspace(-np.pi, np.pi, 360)
        
        # 目标
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_reached = False
        self.goal_failed = False
        self.result_received = False
        
        # 训练参数
        self.state_dim = 28
        self.action_dim = 5
        self.max_steps = 500
        self.goal_threshold = 0.3
        self.collision_threshold = 0.2
        self.time_step = 0.1
        
        # 状态历史
        self.prev_distance = 0.0
        self.prev_action = np.zeros(self.action_dim)
        
        # 等待消息
        rospy.sleep(1.0)
        
        rospy.loginfo(f"状态维度: {self.state_dim}, 动作维度: {self.action_dim}")
        rospy.loginfo("DWA训练环境初始化完成")
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # 从四元数中提取航向
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_yaw = yaw
        
        # 速度
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z
    
    def scan_callback(self, msg):
        """激光扫描回调"""
        try:
            if msg.ranges is None or len(msg.ranges) == 0:
                rospy.logwarn("空激光数据")
                return
                
            ranges = np.array(msg.ranges, dtype=np.float32)
            
            # 处理无效值
            ranges = np.where(np.isnan(ranges), msg.range_max, ranges)
            ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
            
            # 限制范围
            self.laser_ranges = np.clip(ranges, msg.range_min, msg.range_max)
            
            # 计算角度
            if len(self.laser_angles) != len(ranges):
                angle_min = msg.angle_min
                angle_max = msg.angle_max
                self.laser_angles = np.linspace(angle_min, angle_max, len(ranges))
                
        except Exception as e:
            rospy.logwarn(f"激光数据处理错误: {e}")
            self.laser_ranges = np.ones(360, dtype=np.float32) * 3.5
            self.laser_angles = np.linspace(-np.pi, np.pi, 360)
    
    def get_distance_to_goal(self):
        """计算到目标的距离"""
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_relative_angle(self):
        """计算相对目标的角度"""
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        if dx == 0 and dy == 0:
            return 0.0
        
        goal_angle = math.atan2(dy, dx)
        relative_angle = goal_angle - self.robot_yaw
        return math.atan2(math.sin(relative_angle), math.cos(relative_angle))
    
    def get_state(self):
        """获取当前状态"""
        state = []
        
        # 1. 激光数据 (24维)
        if self.laser_ranges is not None and len(self.laser_ranges) > 0:
            # 均匀采样24个激光束
            num_beams = 24
            if len(self.laser_ranges) >= num_beams:
                indices = np.linspace(0, len(self.laser_ranges)-1, num_beams, dtype=int)
                sampled_ranges = self.laser_ranges[indices]
            else:
                sampled_ranges = np.ones(num_beams) * 3.5
        else:
            sampled_ranges = np.ones(24) * 3.5
        
        # 归一化
        sampled_ranges = np.clip(sampled_ranges, 0.1, 3.5) / 3.5
        state.extend(sampled_ranges.tolist())
        
        # 2. 目标信息
        distance = self.get_distance_to_goal()
        angle = self.get_relative_angle()
        
        # 归一化
        distance = np.clip(distance, 0.0, 5.0) / 5.0
        angle = angle / math.pi
        state.append(distance)
        state.append(angle)
        
        # 3. 速度
        state.append(np.clip(self.robot_vx, -0.5, 0.5) / 0.5)
        state.append(np.clip(self.robot_wz, -1.0, 1.0) / 1.0)
        
        return np.array(state, dtype=np.float32)
    
    def reset(self, test_mode=False, episode=None):
        """重置环境"""
        rospy.loginfo("重置环境...")
        
        # 重置标志
        self.goal_reached = False
        self.goal_failed = False
        self.result_received = False
        
        # 停止机器人
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        
        # 重置机器人位置
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            rospy.loginfo("仿真世界已重置")
        except rospy.ServiceException as e:
            rospy.logwarn(f"无法重置仿真世界: {e}")
        except rospy.ROSException as e:
            rospy.logwarn(f"无法找到重置服务: {e}")
        
        # 等待位置重置
        rospy.sleep(1.0)
        
        # 重置机器人状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        
        # 设置新目标
        if test_mode:
            # 测试模式：固定目标
            self.goal_x = 3.0
            self.goal_y = 0.0
        else:
            # 训练模式：随机目标
            r = np.random.uniform(2.0, 4.0)
            theta = np.random.uniform(-math.pi/2, math.pi/2)
            self.goal_x = r * math.cos(theta)
            self.goal_y = r * math.sin(theta)
        
        rospy.loginfo(f"新目标: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        
        # 发送目标
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        
        # 设置目标朝向
        angle = np.random.uniform(-math.pi, math.pi)
        q = quaternion_from_euler(0, 0, angle)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        self.goal_pub.publish(goal)
        
        # 记录初始距离
        self.prev_distance = self.get_distance_to_goal()
        self.prev_action = np.zeros(self.action_dim)
        
        rospy.sleep(0.5)
        return self.get_state()
    
    def _check_collision(self):
        """检查碰撞"""
        if self.laser_ranges is not None and len(self.laser_ranges) > 0:
            min_distance = np.min(self.laser_ranges)
            return min_distance < self.collision_threshold
        return False
    
    def _check_goal(self):
        """检查是否到达目标"""
        distance = self.get_distance_to_goal()
        return distance < self.goal_threshold
    
    def get_reward(self, action):
        """计算奖励"""
        reward = 0.0
        done = False
        info = {'success': False, 'reason': 'running'}
        
        # 获取当前状态
        distance = self.get_distance_to_goal()
        angle = self.get_relative_angle()
        
        # 1. 到达目标
        if self._check_goal():
            reward += 100.0
            done = True
            info['success'] = True
            info['reason'] = 'goal_reached'
            rospy.loginfo("到达目标!")
            return reward, done, info
        
        # 2. 碰撞检测
        if self._check_collision():
            reward -= 50.0
            done = True
            info['success'] = False
            info['reason'] = 'collision'
            rospy.logwarn("碰撞!")
            return reward, done, info
        
        # 3. 距离奖励
        distance_improvement = self.prev_distance - distance
        reward += distance_improvement * 20.0
        
        # 4. 朝向奖励
        angle_penalty = -abs(angle) * 5.0
        reward += angle_penalty
        
        # 5. 速度奖励
        if self.robot_vx > 0.0:
            reward += self.robot_vx * 2.0
        
        # 6. 平滑惩罚
        action_change = np.sum(np.abs(action - self.prev_action))
        reward -= action_change * 0.1
        
        # 7. 步数惩罚
        reward -= 0.05
        
        # 更新
        self.prev_distance = distance
        self.prev_action = action.copy()
        
        return reward, done, info
    
    def set_dwa_params(self, action):
        """设置DWA参数"""
        # 从动作映射到DWA参数 [-1, 1] -> [实际范围]
        params = {
            'max_vel_x': float(np.clip(0.2 + (action[0] + 1) * 0.3, 0.1, 0.8)),  # 0.1-0.8
            'min_vel_x': float(np.clip(-0.1 + action[1] * 0.2, -0.3, 0.0)),      # -0.3-0.0
            'max_rot_vel': float(np.clip(0.5 + (action[2] + 1) * 0.75, 0.5, 2.0)),  # 0.5-2.0
            'acc_lim_x': float(np.clip(0.5 + (action[3] + 1) * 0.75, 0.5, 2.0)),  # 0.5-2.0
            'vx_samples': int(10 + (action[4] + 1) * 20)                   # 10-50
        }
        
        # 设置参数
        rospy.set_param('/move_base/DWAPlannerROS/max_vel_x', params['max_vel_x'])
        rospy.set_param('/move_base/DWAPlannerROS/min_vel_x', params['min_vel_x'])
        rospy.set_param('/move_base/DWAPlannerROS/max_rot_vel', params['max_rot_vel'])
        rospy.set_param('/move_base/DWAPlannerROS/acc_lim_x', params['acc_lim_x'])
        rospy.set_param('/move_base/DWAPlannerROS/vx_samples', params['vx_samples'])
        
        rospy.logdebug(f"DWA参数: {params}")
        return params
    
    def step(self, action):
        """执行一步"""
        # 设置DWA参数
        params = self.set_dwa_params(action)
        
        # 等待DWA控制
        rospy.sleep(self.time_step)
        
        # 计算奖励
        reward, done, info = self.get_reward(action)
        
        # 获取新状态
        state = self.get_state()
        
        return state, reward, done, info

# ==================== 训练器 ====================
class DWARL_Trainer:
    """DWA RL训练器"""
    def __init__(self, config):
        rospy.loginfo("初始化DWA RL训练器...")
        
        # 配置
        self.config = config
        self.episodes = config.get('episodes', 1000)
        self.max_steps = config.get('max_steps', 500)
        self.batch_size = config.get('batch_size', 256)
        self.warmup_steps = config.get('warmup_steps', 1000)
        self.save_freq = config.get('save_freq', 50)
        self.log_interval = config.get('log_interval', 10)
        self.device = torch.device(config.get('device', 'cuda' if torch.cuda.is_available() else 'cpu'))
        
        # 创建环境
        rospy.loginfo("创建环境...")
        self.env = DWATrainingEnv()
        
        # 创建智能体
        rospy.loginfo("创建智能体...")
        self.agent = DWA_RL_Agent(
            state_dim=self.env.state_dim,
            action_dim=self.env.action_dim,
            device=self.device
        )
        
        # 训练记录
        self.episode_rewards = []
        self.episode_steps = []
        self.episode_successes = []
        
        # 保存目录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = config.get('save_dir', f'./dwa_training_{timestamp}')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 日志文件
        self.log_file = os.path.join(self.save_dir, 'training_log.csv')
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'episode', 'steps', 'reward', 'success', 'distance',
                'avg_reward_10', 'success_rate_10', 'actor_loss', 'critic_loss',
                'noise_scale', 'memory_size', 'time'
            ])
        
        rospy.loginfo("="*60)
        rospy.loginfo("DWA RL训练器初始化完成")
        rospy.loginfo(f"训练回合: {self.episodes}")
        rospy.loginfo(f"每回合最大步数: {self.max_steps}")
        rospy.loginfo(f"批次大小: {self.batch_size}")
        rospy.loginfo(f"热身步数: {self.warmup_steps}")
        rospy.loginfo(f"保存频率: {self.save_freq}")
        rospy.loginfo(f"设备: {self.device}")
        rospy.loginfo(f"保存目录: {self.save_dir}")
        rospy.loginfo("="*60)
    
    def train(self):
        """主训练循环"""
        rospy.loginfo("开始训练...")
        start_time = time.time()
        
        try:
            for episode in range(self.episodes):
                if rospy.is_shutdown():
                    rospy.logwarn("收到ROS关闭信号，终止训练")
                    break
                
                episode_start = time.time()
                
                # 重置环境
                state = self.env.reset(episode=episode)
                episode_reward = 0.0
                episode_steps = 0
                episode_success = False
                episode_distance = 0.0
                actor_losses = []
                critic_losses = []
                
                for step in range(self.max_steps):
                    if rospy.is_shutdown():
                        break
                    
                    # 选择动作
                    if self.agent.total_steps < self.warmup_steps:
                        action = np.random.uniform(-1.0, 1.0, self.env.action_dim)
                    else:
                        action = self.agent.select_action(state, add_noise=True)
                    
                    # 执行动作
                    next_state, reward, done, info = self.env.step(action)
                    
                    # 存储经验
                    self.agent.remember(state, action, reward, next_state, done)
                    
                    # 训练
                    if len(self.agent.memory) >= self.batch_size and self.agent.total_steps % 4 == 0:
                        actor_loss, critic_loss = self.agent.train()
                        if actor_loss is not None:
                            actor_losses.append(actor_loss)
                        if critic_loss is not None:
                            critic_losses.append(critic_loss)
                    
                    # 更新
                    state = next_state
                    episode_reward += reward
                    episode_steps += 1
                    self.agent.total_steps += 1
                    
                    if done:
                        episode_success = info.get('success', False)
                        episode_distance = self.env.get_distance_to_goal()
                        rospy.loginfo(f"回合结束: 原因={info.get('reason', 'unknown')}, 成功={episode_success}")
                        break
                
                # 记录回合
                elapsed = time.time() - episode_start
                self.episode_rewards.append(episode_reward)
                self.episode_steps.append(episode_steps)
                self.episode_successes.append(episode_success)
                
                # 计算统计
                avg_reward_10 = np.mean(self.episode_rewards[-10:]) if len(self.episode_rewards) >= 10 else episode_reward
                success_rate_10 = np.mean(self.episode_successes[-10:]) if len(self.episode_successes) >= 10 else 0.0
                avg_actor_loss = np.mean(actor_losses) if actor_losses else 0.0
                avg_critic_loss = np.mean(critic_losses) if critic_losses else 0.0
                memory_size = len(self.agent.memory)
                noise_scale = self.agent.noise_scale
                
                # 记录日志
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        episode+1, episode_steps, f"{episode_reward:.4f}", 1 if episode_success else 0,
                        f"{episode_distance:.4f}", f"{avg_reward_10:.4f}", f"{success_rate_10:.4f}",
                        f"{avg_actor_loss:.6f}", f"{avg_critic_loss:.6f}",
                        f"{noise_scale:.4f}", memory_size, f"{elapsed:.2f}"
                    ])
                
                # 打印进度
                if (episode + 1) % self.log_interval == 0 or episode_success or episode < 5:
                    rospy.loginfo(f"回合 {episode+1:4d}/{self.episodes} | "
                                 f"奖励: {episode_reward:7.2f} | "
                                 f"步数: {episode_steps:3d} | "
                                 f"成功: {'✓' if episode_success else '✗'} | "
                                 f"平均奖励: {avg_reward_10:7.2f} | "
                                 f"成功率: {success_rate_10 * 100:5.1f}% | "
                                 f"损失: {avg_critic_loss:6.4f} | "
                                 f"噪声: {noise_scale:.3f} | "
                                 f"时间: {elapsed:.1f}s")
                
                # 保存检查点
                if (episode + 1) % self.save_freq == 0 or episode_success:
                    self.save_checkpoint(episode+1)
                
                # 清理内存
                if episode > 0 and episode % 100 == 0:
                    torch.cuda.empty_cache() if torch.cuda.is_available() else None
                
                # 添加短暂休息避免过载
                rospy.sleep(0.5)
        
        except KeyboardInterrupt:
            rospy.logwarn("训练被用户中断")
        except Exception as e:
            rospy.logerr(f"训练错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
        finally:
            # 保存最终模型
            self.save_checkpoint('final')
            rospy.loginfo(f"训练完成! 总时间: {(time.time()-start_time)/60:.1f}分钟")
            
            # 保存训练统计
            self.save_training_stats()
    
    def save_checkpoint(self, episode):
        """保存检查点"""
        checkpoint_path = os.path.join(self.save_dir, f'checkpoint_episode_{episode:04d}.pth')
        torch.save({
            'episode': episode,
            'actor_state_dict': self.agent.actor.state_dict(),
            'critic_state_dict': self.agent.critic.state_dict(),
            'actor_target_state_dict': self.agent.actor_target.state_dict(),
            'critic_target_state_dict': self.agent.critic_target.state_dict(),
            'actor_optimizer_state_dict': self.agent.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.agent.critic_optimizer.state_dict(),
            'episode_rewards': self.episode_rewards,
            'episode_steps': self.episode_steps,
            'episode_successes': self.episode_successes,
            'actor_losses': self.agent.actor_losses,
            'critic_losses': self.agent.critic_losses,
        }, checkpoint_path)
        rospy.loginfo(f"检查点保存到: {checkpoint_path}")
    
    def save_training_stats(self):
        """保存训练统计"""
        stats_path = os.path.join(self.save_dir, 'training_stats.json')
        stats = {
            'total_episodes': len(self.episode_rewards),
            'total_steps': self.agent.total_steps,
            'episode_rewards': [float(r) for r in self.episode_rewards],
            'episode_steps': [int(s) for s in self.episode_steps],
            'episode_successes': [bool(s) for s in self.episode_successes],
            'actor_losses': [float(l) for l in self.agent.actor_losses],
            'critic_losses': [float(l) for l in self.agent.critic_losses],
            'average_reward': float(np.mean(self.episode_rewards)) if self.episode_rewards else 0.0,
            'success_rate': float(np.mean(self.episode_successes)) if self.episode_successes else 0.0,
            'average_steps': float(np.mean(self.episode_steps)) if self.episode_steps else 0.0,
        }
        
        with open(stats_path, 'w') as f:
            json.dump(stats, f, indent=2)
        rospy.loginfo(f"训练统计保存到: {stats_path}")

# ==================== 主函数 ====================
def main():
    """主函数"""
    rospy.loginfo("启动DWA RL训练")
    
    # 检查PyTorch
    rospy.loginfo(f"PyTorch版本: {torch.__version__}")
    rospy.loginfo(f"CUDA可用: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        rospy.loginfo(f"GPU设备: {torch.cuda.get_device_name(0)}")
    
    # 训练配置
    config = {
        'episodes': 1000,
        'max_steps': 500,
        'batch_size': 256,
        'warmup_steps': 1000,
        'save_freq': 50,
        'log_interval': 10,
        'device': 'cuda' if torch.cuda.is_available() else 'cpu',
        'save_dir': './dwa_training_results',
    }
    
    # 创建训练器
    trainer = DWARL_Trainer(config)
    
    # 启动训练
    trainer.train()
    
    rospy.loginfo("DWA RL训练结束")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断")
    except Exception as e:
        rospy.logerr(f"主程序错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())