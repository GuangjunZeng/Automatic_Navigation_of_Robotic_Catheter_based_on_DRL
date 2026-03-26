# 环境配置部分（强制使用Gymnasium接口）
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback
import math
import torch
import torch.nn as nn

class CatheterEnv(gym.Env):
    """
    自定义导管尖端导航环境
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, env_id=0, render_mode=None):
        super().__init__()

        # 初始化关键属性
        self.catheter_tip_theta = 0.0  # 初始方向
        self.catheter_tip_radius = 5  # 导管尖端半径
        self.dt = 1  # 时间步长（秒）
        self.env_id = env_id  # 添加环境ID

        # 根据 III-A1节初始化参数
        self.env_width = 110  # 环境宽度（米）
        self.env_height = 150  # 环境高度
        self.num_obstacles = 3  # 初始障碍物数量
        self.max_steps = 800  # 最大时间步

        # 根据 表II设置随机化范围
        self.param_ranges = {
            'catheter_tip_speed': (7, 12),
            'num_obstacles': (3, 4),
            'goal_radius': (3, 8)
        }

        # 固定参数
        self.max_obstacles = 4  # 根据表II的最大障碍物数量
        self.base_obs_dim = 8  # px,py,θ,r,gx,gy,do,t
        self.obstacle_obs_dim = 3 * self.max_obstacles  # 每个障碍物3个参数
        self.total_obs_dim = self.base_obs_dim + self.obstacle_obs_dim  # 总维度=8+54=62
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.total_obs_dim,),
            dtype=np.float32)

 
        self.action_space = spaces.Box(
            low= -1, high= 1, shape=(1,), dtype=np.float32)

        # 初始化物理参数: catheter_tip_speed, num_obstacles, goal_radius
        self._randomize_parameters()
        #???这个重复多余了吧？   这个不是固定的，放在init中不合适。但多着也没影响

    def _randomize_parameters(self):

        # 从表II中采样参数
        self.catheter_tip_speed = np.random.uniform(*self.param_ranges['catheter_tip_speed'])
        self.num_obstacles = np.random.randint(*self.param_ranges['num_obstacles'])
        self.goal_radius = np.random.uniform(*self.param_ranges['goal_radius'])

        # 动态调整观察空间形状
        #self.observation_space.shape = (4 + 3 * self.num_obstacles + 2,)

        # 根据 添加传感器噪声参数（表I）
        self.obs_noise = {
            'position': 1e-3,  # 位置噪声
            'orientation': 0.03,  # 方向噪声（弧度）
            'obstacle_pos': 15e-5  # 障碍物位置噪声
        }

    def _generate_single_obstacle(self):
    
        r = np.random.choice([-1, 1])
        if r == -1:  # 底部生成
            x = np.random.uniform(self.env_width / 20, 19 * self.env_width / 20)
            y = np.random.uniform(0, 20 * self.env_height / 30)
        else:  # 右侧生成
            x = np.random.uniform(5 * self.env_width / 20, 19 * self.env_width / 20)
            y = np.random.uniform(20 * self.env_height / 30, 29 * self.env_height / 30)

        # 添加随机速度（公式3）
        speed_x = np.random.uniform(-self.catheter_tip_speed / 4, self.catheter_tip_speed / 4)
        speed_y = np.random.uniform(-self.catheter_tip_speed / 4, self.catheter_tip_speed / 4)
        radius = np.random.uniform(2, 7)

        return {
            'position': np.array([x, y]),
            'velocity': np.array([speed_x, speed_y]),
            'radius': radius
        }

    def _generate_obstacles(self):
        self.obstacles = [
            self._generate_single_obstacle()
            for _ in range(self.num_obstacles)
        ]

    def reset(self, seed=None, options=None):
        """重置环境"""
        super().reset(seed=seed)

        # 随机生成障碍物
        self.num_obstacles = np.random.randint(6, 18)
        self._randomize_parameters()
        self._generate_obstacles()

        # 随机初始化导管尖端位置
        self.catheter_tip_pos = np.array([
            np.random.uniform(0, self.env_width/10),
            np.random.uniform(9*self.env_height/10, self.env_height)
        ])

        #随机初始化goal目标位置
        self.goal_pos = np.array([
            np.random.uniform(9*self.env_width/10, self.env_width),
            np.random.uniform(0, self.env_height/10)
        ])

        # 初始化导管尖端的方向
        catheter_tip_dx = self.goal_pos[0] - self.catheter_tip_pos[0]
        catheter_tip_dy = self.goal_pos[1] - self.catheter_tip_pos[1]
        catheter_tip_initial_theta = np.arctan2(catheter_tip_dy, catheter_tip_dx)  # 初始方向指向目标
        self.catheter_tip_theta = catheter_tip_initial_theta + + np.random.uniform(-np.pi/24, np.pi/24)

        self.current_step = 0

        return self._get_obs(), {}  # 必须返回两个值


    def _get_boundary_distance(self):
     
        distances = [
            self.catheter_tip_pos[0],  # 左边界
            self.env_width - self.catheter_tip_pos[0],  # 右边界
            self.catheter_tip_pos[1],  # 下边界
            self.env_height - self.catheter_tip_pos[1]  # 上边界
        ]
        return np.min(distances)

    def _get_obs(self):

        # 添加传感器噪声（ III-B节）
        noisy_pos = self.catheter_tip_pos + np.random.normal(0, self.obs_noise['position'], 2)
        noisy_theta = self.catheter_tip_theta + np.random.normal(0, self.obs_noise['orientation'])

        # 基础观测部分1
        base_obs1 = [
            self.catheter_tip_pos[0],  # px
            self.catheter_tip_pos[1],  # py
            self.catheter_tip_theta,  # θ
            self.catheter_tip_radius,  # r
            self.goal_pos[0],  # gx
            self.goal_pos[1],  # gy
        ]

        base_obs2 = [
            self._get_boundary_distance(),  # d0
            self.current_step  # 
        ]

        # 障碍物部分（54维=3×18）
        obst_obs = []
        for i in range(self.max_obstacles):
            if i < len(self.obstacles):
                obst = self.obstacles[i]
                obst_obs += [
                    obst['position'][0],  # oix
                    obst['position'][1],  # oiy
                    obst['radius']  # ri
                ]
            else:
                # 无效障碍物填充-1
                obst_obs += [-100.0, -100.0, -100.0]
        obs = base_obs1 + obst_obs + base_obs2
        return np.array(obs, dtype=np.float32)

    def step(self, action):
        """环境步进"""

        delta_theta = action[0] * np.pi / 5  # 限制在±π/6范围内, 这个其实不是很需要???
        self.catheter_tip_theta = (self.catheter_tip_theta + delta_theta) % (2 * np.pi)

        # 计算运动（考虑边界漂移）
        #drift_angle = np.deg2rad(16)     #  III-B节提到的16度漂移
        actual_theta = self.catheter_tip_theta   #+ drift_angle

        actual_theta_degree = math.degrees(actual_theta)
        print("current_step: ", self.current_step, "actual_theta_degree: ", actual_theta_degree)

        # 添加动作噪声（ III-B节）
        actual_theta += np.random.normal(0, 0.01 * abs(delta_theta)+ 1e-3)
        print("actual_theta_degree with noise: ", actual_theta_degree)

        # 更新导管尖端位置
        velocity = self.catheter_tip_speed * np.array([
            np.cos(actual_theta),
            np.sin(actual_theta)
        ])
        self.catheter_tip_pos += velocity * self.dt

        # 更新存在的障碍物位置（动态调整速度和替换越界障碍物）
        new_obstacles = []
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                # === 动态生成速度（公式3） ===
                speed_x = np.random.uniform(
                    -self.catheter_tip_speed / 3,
                    self.catheter_tip_speed / 3
                )
                speed_y = np.random.uniform(
                    -self.catheter_tip_speed / 3,
                    self.catheter_tip_speed / 3
                )
                obst['velocity'] = np.array([speed_x, speed_y])

                # 更新位置
                obst['position'] += obst['velocity'] * self.dt

                # === 检查边界并替换越界障碍物（ III-A1节） ===
                x, y = obst['position']
                if (x < 0 or x > self.env_width or
                        y < 0 or y > self.env_height):
                    # 生成新障碍物（调用生成逻辑）
                    new_obst = self._generate_single_obstacle()
                    new_obstacles.append(new_obst)
                else:
                    new_obstacles.append(obst)
            else:
                new_obstacles.append(obst)

        self.obstacles = new_obstacles

        # 检查碰撞
        #done = self._check_collision() or (self.current_step >= self.max_steps)

        # 计算终止条件
        terminated_collision = self._check_collision()
        distance_to_goal = np.linalg.norm(self.catheter_tip_pos - self.goal_pos)
        terminated_goal = distance_to_goal <= self.goal_radius
        terminated = terminated_goal or terminated_collision
        truncated = (self.current_step >= self.max_steps)

        # 计算奖励（公式6-10）
        reward = self._calculate_reward(velocity)

        # 到达目标给予额外奖励
        if terminated_goal:
            reward += 100  # 成功奖励
        print("actual_reward: ", reward)
        if terminated_collision:
            reward -= 70
        print("actual_reward: ", reward)


        self.current_step += 1
        print(f"Env {self.env_id} - current_step: {self.current_step}", "actual_theta: ", actual_theta)
        # print("current step: ", self.current_step, ";", "actual_theta: ", actual_theta)

        return (
            self._get_obs(), #??? 障碍物的信息对策略的改进没有影响啊？
            reward,
            terminated,  # 明确终止条件
            truncated,  # 明确截断条件
            {}  # 必须返回五个值
        )

    def _check_collision(self):
        """碰撞检测"""
        # 检查边界碰撞
        if (self.catheter_tip_pos[0] < 0 or self.catheter_tip_pos[0] > self.env_width or
                self.catheter_tip_pos[1] < 0 or self.catheter_tip_pos[1] > self.env_height):
            return True

        # 检查障碍物碰撞
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                distance = np.linalg.norm(self.catheter_tip_pos - obst['position'])
                if distance <= (self.catheter_tip_radius + obst['radius']):
                    return True
            else:
                break
        return False


    def _calculate_reward(self, velocity):
        """实现 III-A4节的奖励函数"""
        # 导航奖励（公式7）
        distance_to_goal = np.linalg.norm(self.catheter_tip_pos - self.goal_pos)
        #if distance_to_goal > 100:
            #bn = -0.05 * distance_to_goal
        #else:
        scale = 30
        bn = 10 / ( (abs(distance_to_goal))/scale + 1e-2 )  # ca=0.1


        # 障碍物惩罚（公式8）
        po = 0
        index_obs = 0
        d_safe = 35
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                dist = np.linalg.norm(self.catheter_tip_pos - obst['position']) - (obst['radius']+self.catheter_tip_radius)
                #??? 需不需要只考虑局部的呢?????
                if dist < d_safe :
                    #po -= 1.6 * ( 1/(abs(dist))  -  1/(d_safe) )
                    po -= 5 * ( 1/(abs(dist)) + 1e-3 )
            else:
                break

        # 时间惩罚（公式9）
        pt = -0.01  #??? 我认为这个计算是有问题的?????

        # 速度势能（公式10）
        pv = 0
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                relative_vel = velocity - obst['velocity']
                obstacle_to_catheter_tip = self.catheter_tip_pos - obst['position']
                unit_vector = obstacle_to_catheter_tip / (np.linalg.norm(obstacle_to_catheter_tip) + 1e-8)
                dot_product = np.dot(relative_vel, unit_vector)
                if dot_product < 0:
                    dist = np.linalg.norm(self.catheter_tip_pos - obst['position']) - (obst['radius'] + self.catheter_tip_radius)
                    #??? 只考虑局部障碍物的影响
                    if dist < d_safe:
                        pv += 0.02 * dot_product
            else:
                break
        print('bn: ', bn, '; po: ', po, '; pt: ', pt, '; pv: ', pv)
        return bn + po + pt + pv

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
import torch.nn as nn
from stable_baselines3.common.callbacks import CheckpointCallback



# 训练部分（对应 III-C节）
def train():
    # 创建并行环境
    #env = DummyVecEnv([lambda: CatheterEnv() for _ in range(8)])
    env = CatheterEnv()
    env = DummyVecEnv([lambda: env])  # 单环境

    # 强制使用新版接口
    from stable_baselines3.common.env_checker import check_env
    check_env(env.envs[0])  # 验证单个环境


    # 定义策略网络和价值网络的结构
    policy_kwargs = dict(
        net_arch=[
            #dict(vf=[256, 128], pi=[256, 128]),  # 前几层使用 ReLU
            #dict(vf=[128, 64], pi=[128, 64])  # 后几层使用 RBF
        ],
        activation_fn=nn.GELU,  # 统一使用GELU激活
        ortho_init=True,
    )

    # 定义SAC模型（使用MlpPolicy）
    model = SAC(
        "MlpPolicy",
        env,
        #policy_kwargs=policy_kwargs,  # 共享网络结构
        verbose=1,
        learning_rate=1e-3,  # 保持相同学习率
        buffer_size=1_000_000,  # SAC特有：经验回放缓冲区大小（建议1e6起）
        batch_size=256,  # 保持相同batch_size
        tau=0.005,  # SAC特有：目标网络更新系数（默认0.005）
        gamma=0.87,  # 保持相同折扣因子
        ent_coef='auto',  # SAC核心：自动调整熵系数（探索权重）
        target_entropy='auto',  # 自动计算目标熵
        train_freq=1,  # 每步训练一次（可改为元组(1, 'step')或(1, 'episode')）
        gradient_steps=9,  # 近似PPO的n_epochs（每个batch训练次数）
        device='cuda'
    )

    # 设置评估回调
    eval_callback = EvalCallback(
        env,
        best_model_save_path='./logs/',
        log_path='./logs/',
        eval_freq=50000,
        deterministic=True,
        render=True
    )

    # 添加CheckpointCallback以定期保存模型
    checkpoint_callback = CheckpointCallback(
        save_freq=50000,  # 每隔50000步保存一次模型
        save_path='C:\\Users\\admin\\Desktop\\IML\\capstone-2025\\深度强化学习模型\\models\\',  # 保存路径
        name_prefix='sac_catheter_tip_0409_{total_timesteps}',  # 文件名前缀
        save_replay_buffer=False,  # 是否保存经验回放缓冲区（可选）
        save_vecnormalize=False  # 如果使用VecNormalize，是否保存归一化参数（可选）
    )

    # 开始训练（1M steps）
    model.learn(
        total_timesteps=500000,
        callback=[eval_callback, checkpoint_callback],
        progress_bar=True
    )

    # 保存模型
    model.save("sac_catheter_tip_raw_0409")  # 生成 ppo_catheter_tip_raw.zip
    #env.save("vec_normalize.pkl")


if __name__ == "__main__":
    # 训练模型
    train()
    print("finish training !!!")

