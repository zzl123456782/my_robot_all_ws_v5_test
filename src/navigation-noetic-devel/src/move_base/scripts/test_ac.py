import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import matplotlib.pyplot as plt
import random
import time

# 设置随机种子
np.random.seed(42)
torch.manual_seed(42)
random.seed(42)

# ==================== 1. 修复的MountainCar环境 ====================
class MountainCar:
    """修复的小车爬坡环境"""
    def __init__(self):
        self.min_position = -1.2
        self.max_position = 0.6
        self.max_speed = 0.07
        self.goal_position = 0.5
        self.actions = [-1, 0, 1]  # 左, 不动, 右
        self.position = None
        self.velocity = None
        
    def reset(self):
        """重置到随机位置"""
        self.position = np.random.uniform(-0.6, -0.4)  # 随机起始位置
        self.velocity = 0.0
        return self.get_state()
    
    def get_state(self):
        """获取状态（归一化到[-1,1]）"""
        # 归一化
        pos_norm = (self.position - self.min_position) / (self.max_position - self.min_position) * 2 - 1
        vel_norm = self.velocity / self.max_speed
        return np.array([pos_norm, vel_norm], dtype=np.float32)
    
    def step(self, action_idx):
        """执行一步"""
        action_idx = int(action_idx) % 3
        force = self.actions[action_idx]
        
        # 物理更新
        self.velocity += force * 0.001 - 0.0025 * np.cos(3 * self.position)
        if self.velocity > self.max_speed:
            self.velocity = self.max_speed
        if self.velocity < -self.max_speed:
            self.velocity = -self.max_speed
        
        self.position += self.velocity
        if self.position > self.max_position:
            self.position = self.max_position
        if self.position < self.min_position:
            self.position = self.min_position
            if self.velocity < 0:
                self.velocity = 0
        
        # 计算奖励
        done = False
        reward = -1.0  # 每步惩罚，鼓励快速到达
        
        # 到达目标
        if self.position >= self.goal_position:
            done = True
            reward = 100.0
        # 靠近目标奖励
        elif self.position > 0.4:
            reward += 0.5
        # 速度奖励
        if self.velocity > 0.05:
            reward += 0.1
        
        return self.get_state(), reward, done, {}

# ==================== 2. Actor网络 ====================
class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Softmax(dim=-1)
        )
        
    def forward(self, x):
        return self.net(x)

# ==================== 3. Critic网络 ====================
class Critic(nn.Module):
    def __init__(self, state_dim, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )
        
    def forward(self, x):
        return self.net(x)

# ==================== 4. AC智能体 ====================
class ACAgent:
    def __init__(self, state_dim, action_dim, lr_actor=0.001, lr_critic=0.002, gamma=0.99):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        
        # 网络
        self.actor = Actor(state_dim, action_dim)
        self.critic = Critic(state_dim)
        
        # 优化器
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)
        
        # 探索
        self.epsilon = 1.0
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.05
        
        # 记录
        self.actor_losses = []
        self.critic_losses = []
        
    def get_action(self, state, epsilon=None):
        """选择动作"""
        if epsilon is None:
            epsilon = self.epsilon
            
        # ε-greedy探索
        if np.random.random() < epsilon:
            return np.random.randint(self.action_dim)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            action_probs = self.actor(state_tensor)
        
        # 采样动作
        action = torch.multinomial(action_probs, 1).item()
        return action
    
    def train(self, states, actions, rewards, next_states, dones):
        """训练一步"""
        # 转换为张量
        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(next_states)
        dones = torch.BoolTensor(dones)
        
        # 计算TD目标
        with torch.no_grad():
            next_values = self.critic(next_states).squeeze()
            # 如果done，未来价值为0
            targets = rewards + self.gamma * next_values * (~dones)
        
        # 计算当前价值
        current_values = self.critic(states).squeeze()
        
        # 计算优势
        advantages = targets - current_values
        
        # 训练Critic
        self.critic_optimizer.zero_grad()
        critic_loss = F.mse_loss(current_values, targets)
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 1.0)
        self.critic_optimizer.step()
        
        # 重新计算价值（Critic更新后）
        with torch.no_grad():
            current_values = self.critic(states).squeeze()
            advantages = targets - current_values
        
        # 训练Actor
        self.actor_optimizer.zero_grad()
        
        # 获取动作概率
        action_probs = self.actor(states)
        selected_action_probs = action_probs[torch.arange(len(actions)), actions]
        
        # 策略梯度损失
        actor_loss = -(torch.log(selected_action_probs + 1e-8) * advantages).mean()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 1.0)
        self.actor_optimizer.step()
        
        # 记录损失
        self.actor_losses.append(actor_loss.item())
        self.critic_losses.append(critic_loss.item())
        
        # 衰减探索率
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        
        return actor_loss.item(), critic_loss.item()

# ==================== 5. 经验回放缓冲区 ====================
class ReplayBuffer:
    def __init__(self, capacity=10000):
        self.buffer = []
        self.capacity = capacity
        self.position = 0
        
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity
        
    def sample(self, batch_size):
        batch = random.sample(self.buffer, min(batch_size, len(self.buffer)))
        states, actions, rewards, next_states, dones = zip(*batch)
        return np.array(states), np.array(actions), np.array(rewards), np.array(next_states), np.array(dones)
    
    def __len__(self):
        return len(self.buffer)

# ==================== 6. 训练函数 ====================
def train_agent(episodes=500, max_steps=200, batch_size=64):
    """训练智能体"""
    env = MountainCar()
    agent = ACAgent(state_dim=2, action_dim=3)
    buffer = ReplayBuffer(capacity=10000)
    
    episode_rewards = []
    episode_lengths = []
    success_rate_history = []
    
    print("开始训练...")
    print("Episode | Reward  | Steps | Epsilon | Status")
    print("-" * 50)
    
    for episode in range(1, episodes + 1):
        state = env.reset()
        episode_reward = 0
        states, actions, rewards, next_states, dones = [], [], [], [], []
        
        for step in range(max_steps):
            # 选择动作
            action = agent.get_action(state)
            
            # 执行动作
            next_state, reward, done, _ = env.step(action)
            episode_reward += reward
            
            # 存储经验
            buffer.push(state, action, reward, next_state, done)
            
            # 添加到batch
            states.append(state)
            actions.append(action)
            rewards.append(reward)
            next_states.append(next_state)
            dones.append(done)
            
            # 更新状态
            state = next_state
            
            if done:
                break
        
        # 记录
        episode_rewards.append(episode_reward)
        episode_lengths.append(step + 1)
        
        # 从经验池采样训练
        if len(buffer) >= batch_size:
            batch_states, batch_actions, batch_rewards, batch_next_states, batch_dones = buffer.sample(batch_size)
            actor_loss, critic_loss = agent.train(batch_states, batch_actions, batch_rewards, batch_next_states, batch_dones)
        
        # 衰减探索率
        agent.epsilon = max(agent.epsilon_min, agent.epsilon * agent.epsilon_decay)
        
        # 计算成功率
        if len(episode_rewards) >= 100:
            recent_rewards = episode_rewards[-100:]
            success_rate = np.mean([1 if r > 0 else 0 for r in recent_rewards]) * 100
            success_rate_history.append(success_rate)
        
        # 打印进度
        if episode % 20 == 0 or episode == 1 or episode_reward > 0:
            status = "GOAL!" if episode_reward > 0 else "exploring"
            print(f"{episode:7d} | {episode_reward:6.1f} | {step+1:5d} | {agent.epsilon:.4f} | {status}")
            
            # 如果是成功的episode，显示详细信息
            if episode_reward > 0:
                print(f"  -> 位置: {env.position:.3f}, 速度: {env.velocity:.3f}")
        
        # 如果连续100轮成功率>90%，提前停止
        if len(success_rate_history) >= 10 and np.mean(success_rate_history[-10:]) > 90:
            print(f"在第{episode}轮提前终止训练，已收敛！")
            break
    
    return agent, episode_rewards, episode_lengths, success_rate_history

# ==================== 7. 可视化 ====================
def plot_training_results(episode_rewards, episode_lengths, success_rate_history, agent):
    """绘制训练结果"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # 1. 奖励曲线
    ax1 = axes[0, 0]
    ax1.plot(episode_rewards, 'b-', alpha=0.6, linewidth=1)
    ax1.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Goal Reward')
    ax1.set_xlabel('Episode')
    ax1.set_ylabel('Total Reward')
    ax1.set_title('Reward per Episode')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # 移动平均
    if len(episode_rewards) > 20:
        window = 20
        moving_avg = np.convolve(episode_rewards, np.ones(window)/window, mode='valid')
        ax1.plot(range(window-1, len(episode_rewards)), moving_avg, 'r-', linewidth=2, label=f'{window}-ep avg')
    
    # 2. 步数曲线
    ax2 = axes[0, 1]
    ax2.plot(episode_lengths, 'g-', alpha=0.6, linewidth=1)
    ax2.set_xlabel('Episode')
    ax2.set_ylabel('Steps')
    ax2.set_title('Steps per Episode')
    ax2.grid(True, alpha=0.3)
    
    # 3. 成功率
    ax3 = axes[0, 2]
    if success_rate_history:
        episodes = range(100, 100 + len(success_rate_history))
        ax3.plot(episodes, success_rate_history, 'purple', linewidth=2)
        ax3.axhline(y=90, color='r', linestyle='--', alpha=0.5, label='90% Success')
        ax3.set_xlabel('Episode')
        ax3.set_ylabel('Success Rate (%)')
        ax3.set_title('Success Rate (Last 100 Episodes)')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
    else:
        ax3.text(0.5, 0.5, 'Need at least 100 episodes\nfor success rate', 
                ha='center', va='center', transform=ax3.transAxes)
        ax3.set_title('Success Rate')
    
    # 4. 策略热图
    ax4 = axes[1, 0]
    positions = np.linspace(-1.2, 0.6, 20)
    best_actions = []
    action_probs_all = []
    
    for pos in positions:
        state = np.array([(pos + 1.2) / 1.8 * 2 - 1, 0.0], dtype=np.float32)  # 归一化
        with torch.no_grad():
            probs = agent.actor(torch.FloatTensor(state).unsqueeze(0)).numpy()[0]
        best_action = np.argmax(probs)
        best_actions.append(best_action)
        action_probs_all.append(probs)
    
    # 绘制动作偏好
    action_colors = ['red', 'gray', 'blue']
    for action_val in [0, 1, 2]:
        mask = np.array(best_actions) == action_val
        if np.any(mask):
            ax4.scatter(np.array(positions)[mask], 
                       [action_val] * np.sum(mask), 
                       c=action_colors[action_val], 
                       label=['Left(-1)', 'Stay(0)', 'Right(1)'][action_val], 
                       alpha=0.6, s=50)
    
    ax4.set_xlabel('Position')
    ax4.set_ylabel('Best Action')
    ax4.set_title('Policy at Zero Velocity')
    ax4.set_yticks([0, 1, 2])
    ax4.set_yticklabels(['Left(-1)', 'Stay(0)', 'Right(1)'])
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. 动作概率分布
    ax5 = axes[1, 1]
    action_probs_all = np.array(action_probs_all)
    width = 0.25
    x = np.arange(len(positions))
    ax5.bar(x - width, action_probs_all[:, 0], width, label='Left(-1)', alpha=0.7, color='red')
    ax5.bar(x, action_probs_all[:, 1], width, label='Stay(0)', alpha=0.7, color='gray')
    ax5.bar(x + width, action_probs_all[:, 2], width, label='Right(1)', alpha=0.7, color='blue')
    ax5.set_xlabel('Position')
    ax5.set_ylabel('Action Probability')
    ax5.set_title('Action Probability Distribution')
    ax5.set_xticks(x[::2])
    ax5.set_xticklabels([f'{p:.1f}' for p in positions[::2]])
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 6. 学习统计
    ax6 = axes[1, 2]
    ax6.axis('off')
    total_episodes = len(episode_rewards)
    success_count = sum(1 for r in episode_rewards if r > 0)
    success_rate = (success_count / total_episodes * 100) if total_episodes > 0 else 0
    
    stats_text = f"训练结果统计:\n\n"
    stats_text += f"总轮数: {total_episodes}\n"
    stats_text += f"成功轮数: {success_count}\n"
    stats_text += f"成功率: {success_rate:.1f}%\n"
    stats_text += f"平均奖励: {np.mean(episode_rewards):.2f}\n"
    stats_text += f"最大奖励: {np.max(episode_rewards):.2f}\n"
    stats_text += f"最小奖励: {np.min(episode_rewards):.2f}\n"
    stats_text += f"平均步数: {np.mean(episode_lengths):.1f}\n"
    stats_text += f"最终探索率: {agent.epsilon:.4f}"
    
    if len(episode_rewards) >= 20:
        last_20_avg = np.mean(episode_rewards[-20:])
        last_20_success = sum(1 for r in episode_rewards[-20:] if r > 0) / 20 * 100
        stats_text += f"\n\n最近20轮:\n"
        stats_text += f"平均奖励: {last_20_avg:.2f}\n"
        stats_text += f"成功率: {last_20_success:.1f}%"
    
    ax6.text(0.1, 0.5, stats_text, fontsize=12, verticalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.show()

# ==================== 8. 测试函数 ====================
def test_agent(agent, num_tests=5):
    """测试智能体"""
    env = MountainCar()
    print("\n" + "="*60)
    print("测试智能体...")
    print("-"*60)
    print(f"{'Test':<6} {'Reward':<10} {'Steps':<8} {'Status':<10}")
    print("-"*60)
    
    for test_idx in range(num_tests):
        state = env.reset()
        episode_reward = 0
        positions = [env.position]
        actions = []
        
        for step in range(200):
            # 无探索
            with torch.no_grad():
                action_probs = agent.actor(torch.FloatTensor(state).unsqueeze(0))
                action = torch.argmax(action_probs).item()
            
            state, reward, done, _ = env.step(action)
            
            episode_reward += reward
            positions.append(env.position)
            actions.append(action)
            
            if done:
                break
        
        status = "SUCCESS" if reward > 0 else "TIMEOUT"
        if reward > 0:
            print(f"{test_idx+1:<6} {episode_reward:<10.1f} {step+1:<8} {status:<10} ✓")
        else:
            print(f"{test_idx+1:<6} {episode_reward:<10.1f} {step+1:<8} {status:<10}")
    
    print("="*60)
    return positions, actions

# ==================== 9. 演示函数 ====================
def demonstrate_agent(agent, env, num_episodes=3):
    """演示智能体行为"""
    for ep in range(num_episodes):
        state = env.reset()
        positions = [env.position]
        velocities = [env.velocity]
        actions = []
        
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 4))
        
        for step in range(200):
            with torch.no_grad():
                action_probs = agent.actor(torch.FloatTensor(state).unsqueeze(0)).numpy()[0]
                action = np.argmax(action_probs)
            
            state, reward, done, _ = env.step(action)
            
            positions.append(env.position)
            velocities.append(env.velocity)
            actions.append(action)
            
            if done:
                break
        
        # 轨迹图
        ax1.plot(positions, 'b-', linewidth=2, alpha=0.8)
        ax1.axhline(y=0.5, color='r', linestyle='--', label='Goal (0.5)')
        ax1.axhline(y=-1.2, color='gray', linestyle=':', alpha=0.5)
        ax1.axhline(y=0.6, color='gray', linestyle=':', alpha=0.5)
        ax1.scatter([0], [positions[0]], c='green', s=100, marker='o', label='Start')
        ax1.scatter([len(positions)-1], [positions[-1]], c='red', s=100, marker='x', label='End')
        ax1.set_xlabel('Step')
        ax1.set_ylabel('Position')
        ax1.set_title(f'Test {ep+1}: Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 速度图
        ax2.plot(velocities, 'g-', linewidth=2, alpha=0.8)
        ax2.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        ax2.set_xlabel('Step')
        ax2.set_ylabel('Velocity')
        ax2.set_title('Velocity over Time')
        ax2.grid(True, alpha=0.3)
        
        # 动作图
        if actions:
            ax3.plot(actions, 'r-', linewidth=2, alpha=0.8)
            ax3.set_yticks([0, 1, 2])
            ax3.set_yticklabels(['Left(-1)', 'Stay(0)', 'Right(1)'])
            ax3.set_ylim(-0.5, 2.5)
            ax3.set_xlabel('Step')
            ax3.set_ylabel('Action')
            ax3.set_title('Action Sequence')
            ax3.grid(True, alpha=0.3)
        
        plt.suptitle(f"Test {ep+1}: Steps = {len(positions)-1}, Reward = {reward:.1f}")
        plt.tight_layout()
        plt.show()

# ==================== 10. 主程序 ====================
if __name__ == "__main__":
    print("="*60)
    print("改进的Actor-Critic强化学习演示")
    print("小车爬坡问题")
    print("="*60)
    
    try:
        # 训练参数
        episodes = 500
        print(f"训练轮数: {episodes}")
        print("训练中...")
        
        # 训练
        start_time = time.time()
        agent, rewards, lengths, success_rates = train_agent(episodes=episodes)
        training_time = time.time() - start_time
        
        # 结果分析
        print("\n" + "="*60)
        print("训练结果分析:")
        print(f"训练时间: {training_time:.1f}秒")
        print(f"总轮数: {len(rewards)}")
        print(f"平均奖励: {np.mean(rewards):.2f}")
        print(f"最大奖励: {np.max(rewards):.2f}")
        print(f"最小奖励: {np.min(rewards):.2f}")
        
        success_count = sum(1 for r in rewards if r > 0)
        success_rate = (success_count / len(rewards) * 100) if rewards else 0
        print(f"成功率: {success_rate:.1f}% ({success_count}/{len(rewards)})")
        
        if len(rewards) >= 20:
            last_20_avg = np.mean(rewards[-20:])
            last_20_success = sum(1 for r in rewards[-20:] if r > 0) / 20 * 100
            print(f"最近20轮平均奖励: {last_20_avg:.2f}")
            print(f"最近20轮成功率: {last_20_success:.1f}%")
        
        # 绘制结果
        plot_training_results(rewards, lengths, success_rates, agent)
        
        # 测试
        test_agent(agent, num_tests=5)
        
        # 演示
        print("\n" + "="*60)
        print("智能体演示...")
        print("="*60)
        demonstrate_agent(agent, MountainCar(), num_episodes=3)
        
        # 最终评估
        if success_rate >= 50:
            print("\n✓ 训练成功！小车学会了爬坡")
        elif success_rate > 0:
            print(f"\n○ 部分成功，成功率为{success_rate:.1f}%")
            print("建议: 增加训练轮数到1000以上")
        else:
            print("\n✗ 训练未成功")
            print("建议:")
            print("1. 增加训练轮数到1000-2000")
            print("2. 调整学习率 (actor_lr=0.0005, critic_lr=0.001)")
            print("3. 增加探索率 (epsilon_start=1.0, epsilon_min=0.1)")
            print("4. 增加批次大小 (batch_size=128)")
        
    except KeyboardInterrupt:
        print("\n\n训练被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n程序结束")
    print("="*60)