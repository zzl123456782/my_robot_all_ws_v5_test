import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from collections import deque
import random
import os

class DWA_RL_Agent:
    def __init__(self, state_dim, action_dim, device='cpu'):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = device
        
        # 网络
        self.actor = ActorNetwork(state_dim, action_dim).to(device)
        self.critic = CriticNetwork(state_dim, action_dim).to(device)
        self.target_actor = ActorNetwork(state_dim, action_dim).to(device)
        self.target_critic = CriticNetwork(state_dim, action_dim).to(device)
        
        # 同步目标网络
        self._update_target_networks(tau=1.0)
        
        # 优化器
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=0.001)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=0.002)
        
        # 经验回放
        self.memory = ReplayBuffer(capacity=100000)
        
        # 超参数
        self.gamma = 0.99
        self.tau = 0.005
        self.batch_size = 64
        
        # 探索噪声
        self.noise = OUNoise(action_dim)
        self.noise_scale = 1.0
        self.noise_decay = 0.995
        self.noise_min = 0.1
        
        # 训练记录
        self.actor_losses = []
        self.critic_losses = []
        
    def select_action(self, state, add_noise=True):
        """选择动作"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action = self.actor(state_tensor).cpu().numpy()[0]
        
        # 添加探索噪声
        if add_noise:
            noise = self.noise.sample() * self.noise_scale
            action = action + noise
            action = np.clip(action, -1.0, 1.0)
        
        return action
    
    def remember(self, state, action, reward, next_state, done):
        """存储经验"""
        self.memory.push(state, action, reward, next_state, done)
    
    def train(self):
        """训练网络"""
        if len(self.memory) < self.batch_size:
            return
        
        # 采样批次
        states, actions, rewards, next_states, dones = self.memory.sample(self.batch_size)
        
        # 转换为张量
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.FloatTensor(dones).unsqueeze(1).to(self.device)
        
        # 训练Critic
        with torch.no_grad():
            next_actions = self.target_actor(next_states)
            target_q = self.target_critic(next_states, next_actions)
            target_q = rewards + (1 - dones) * self.gamma * target_q
        
        current_q = self.critic(states, actions)
        critic_loss = F.mse_loss(current_q, target_q)
        
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # 训练Actor
        actor_actions = self.actor(states)
        actor_loss = -self.critic(states, actor_actions).mean()
        
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # 软更新目标网络
        self._update_target_networks(self.tau)
        
        # 记录损失
        self.actor_losses.append(actor_loss.item())
        self.critic_losses.append(critic_loss.item())
        
        # 衰减探索噪声
        self.noise_scale = max(self.noise_min, self.noise_scale * self.noise_decay)
        
        return actor_loss.item(), critic_loss.item()
    
    def _update_target_networks(self, tau):
        """软更新目标网络"""
        for target_param, param in zip(self.target_actor.parameters(), self.actor.parameters()):
            target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)
        
        for target_param, param in zip(self.target_critic.parameters(), self.critic.parameters()):
            target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)
    
    def save_models(self, episode, path='./models'):
        """保存模型"""
        if not os.path.exists(path):
            os.makedirs(path)
        
        torch.save(self.actor.state_dict(), f'{path}/actor_{episode}.pth')
        torch.save(self.critic.state_dict(), f'{path}/critic_{episode}.pth')
        
    def load_models(self, episode, path='./models'):
        """加载模型"""
        self.actor.load_state_dict(torch.load(f'{path}/actor_{episode}.pth'))
        self.critic.load_state_dict(torch.load(f'{path}/critic_{episode}.pth'))
        self._update_target_networks(tau=1.0)

class ActorNetwork(nn.Module):
    """Actor网络：根据状态输出动作（DWA参数调整）"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(ActorNetwork, self).__init__()
        
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, hidden_dim//2)
        self.fc4 = nn.Linear(hidden_dim//2, action_dim)
        
        self.bn1 = nn.BatchNorm1d(hidden_dim)
        self.bn2 = nn.BatchNorm1d(hidden_dim)
        self.bn3 = nn.BatchNorm1d(hidden_dim//2)
        
        # 初始化权重
        self._initialize_weights()
        
    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                nn.init.constant_(m.bias, 0.1)
    
    def forward(self, x):
        x = F.relu(self.bn1(self.fc1(x)))
        x = F.relu(self.bn2(self.fc2(x)))
        x = F.relu(self.bn3(self.fc3(x)))
        x = torch.tanh(self.fc4(x))  # 输出范围[-1, 1]
        return x

class CriticNetwork(nn.Module):
    """Critic网络：评估状态-动作对的Q值"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(CriticNetwork, self).__init__()
        
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
        """只返回Q1值（用于训练Actor）"""
        x = torch.cat([state, action], dim=1)
        q1 = F.relu(self.fc1(x))
        q1 = F.relu(self.fc2(q1))
        q1 = self.fc3(q1)
        return q1

class ReplayBuffer:
    """经验回放缓冲区"""
    def __init__(self, capacity):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            np.array(states),
            np.array(actions),
            np.array(rewards),
            np.array(next_states),
            np.array(dones)
        )
    
    def __len__(self):
        return len(self.buffer)

class OUNoise:
    """Ornstein-Uhlenbeck过程，用于探索"""
    def __init__(self, action_dim, mu=0.0, theta=0.15, sigma=0.2):
        self.action_dim = action_dim
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.state = np.ones(self.action_dim) * self.mu
        self.reset()
    
    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu
    
    def sample(self):
        dx = self.theta * (self.mu - self.state)
        dx += self.sigma * np.random.randn(self.action_dim)
        self.state += dx
        return self.state