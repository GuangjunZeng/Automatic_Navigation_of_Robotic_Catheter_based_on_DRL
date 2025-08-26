# test_microdrill.py
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO,SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback
import matplotlib  # 先导入 matplotlib 模块
matplotlib.use('TkAgg')  # 然后指定后端
import matplotlib.pyplot as plt  # 如果需要使用 pyplot，接着导入
import torch
import torch.nn as nn

# 你的其他代码...

class TestMicroDrillEnv(gym.Env):
    """测试专用环境（添加可视化功能）"""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, render_mode=None):
        super().__init__()

        # 预初始化可视化相关属性
        self.fig = None
        self.ax = None

        # 初始化关键属性
        self.drill_theta = 0.0  # 初始方向
        self.drill_radius = 5  # 钻头半径
        self.dt = 1  # 时间步长（秒）

        # 根据论文III-A1节初始化参数
        self.env_width = 110  # 环境宽度（米）
        self.env_height = 150  # 环境高度
        self.num_obstacles = 3  # 初始障碍物数量
        self.max_steps = 800  # 最大时间步

        # 根据论文表II设置随机化范围
        self.param_ranges = {
            'microdrill_speed': (9, 15),
            'num_obstacles': (3, 4),
            'goal_radius': (3, 8)
        }

        # 固定参数
        self.max_obstacles = 4  # 根据表II的最大障碍物数量
        self.base_obs_dim = 8  # px,py,θ,r,gx,gy,do,t
        self.obstacle_obs_dim = 3 * self.max_obstacles  # 每个障碍物3个参数
        self.total_obs_dim = self.base_obs_dim + self.obstacle_obs_dim  # 总维度=8+54=62
        # 根据论文III-A2节定义观察空间
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.total_obs_dim,),
            dtype=np.float32)

        # 根据论文III-A3节定义动作空间
        self.action_space = spaces.Box(
            low= -1, high= 1, shape=(1,), dtype=np.float32)

        # 初始化物理参数: microdrill_speed, num_obstacles, goal_radius
        self._randomize_parameters()

    def _randomize_parameters(self):
        """实现论文III-B节的域随机化"""
        # 从表II中采样参数
        self.microdrill_speed = np.random.uniform(*self.param_ranges['microdrill_speed'])
        self.num_obstacles = np.random.randint(*self.param_ranges['num_obstacles'])
        self.goal_radius = np.random.uniform(*self.param_ranges['goal_radius'])

        # 动态调整观察空间形状
        # self.observation_space.shape = (4 + 3 * self.num_obstacles + 2,)

        # 根据论文添加传感器噪声参数（表I）
        self.obs_noise = {
            'position': 1e-3,  # 位置噪声
            'orientation': 0.03,  # 方向噪声（弧度）
            'obstacle_pos': 15e-5  # 障碍物位置噪声
        }


    def _generate_single_obstacle(self):
        """ 根据论文公式(2)生成单个障碍物 """
        r = np.random.choice([-1, 1])
        if r == -1:  # 底部生成
            x = np.random.uniform(self.env_width / 20, 19 * self.env_width / 20)
            y = np.random.uniform(0, 20 * self.env_height / 30)
        else:  # 右侧生成
            x = np.random.uniform(5 * self.env_width / 20, 19 * self.env_width / 20)
            y = np.random.uniform(20 * self.env_height / 30, 29 * self.env_height / 30)

        # 添加随机速度（公式3）
        speed_x = np.random.uniform(- self.microdrill_speed / 4, self.microdrill_speed / 4)
        speed_y = np.random.uniform(- self.microdrill_speed / 4, self.microdrill_speed / 4)
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
        # 必须调用父类reset()以确保兼容性
        super().reset(seed=seed)

        # 固定障碍物数量
        self.num_obstacles = 9
        self._randomize_parameters()
        # 生成随机测试环境中的障碍物参数
        self._generate_obstacles()

        # 固定初始条件: 钻头位置和目标位置
        self.drill_pos = np.array([25.0, 95.0])  # 初始位置
        self.goal_pos = np.array([100.0, 35.0])  # 目标位置

        # 初始化微型钻头的方向
        microdrill_dx = self.goal_pos[0] - self.drill_pos[0]
        microdrill_dy = self.goal_pos[1] - self.drill_pos[1]
        microdrill_initial_theta = np.arctan2(microdrill_dy, microdrill_dx)  # 初始方向指向目标
        self.drill_theta = microdrill_initial_theta + + np.random.uniform(-np.pi / 24, np.pi / 24)

        self.current_step = 0

        return self._get_obs(), {}  # 必须返回两个值！


    def _get_boundary_distance(self):
        """计算到最近边界的距离（论文III-A2节do参数）"""
        distances = [
            self.drill_pos[0],  # 左边界
            self.env_width - self.drill_pos[0],  # 右边界
            self.drill_pos[1],  # 下边界
            self.env_height - self.drill_pos[1]  # 上边界
        ]
        return np.min(distances)


    def _get_obs(self):
        """根据公式(4)构造观察向量"""
        # 添加传感器噪声（论文III-B节）
        noisy_pos = self.drill_pos + np.random.normal(0, self.obs_noise['position'], 2)
        noisy_theta = self.drill_theta + np.random.normal(0, self.obs_noise['orientation'])

        base_obs1 = [
            self.drill_pos[0],  # px
            self.drill_pos[1],  # py
            self.drill_theta,  # θ
            self.drill_radius,  # r
            self.goal_pos[0],  # gx
            self.goal_pos[1],  # gy
        ]

        base_obs2 = [
            self._get_boundary_distance(),  # d0
            self.current_step  # ???!!! t未归一化 (归一化)
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
        # 根据论文III-A3节更新方向
        delta_theta = action[0] * np.pi / 5  # 限制在±π/6范围内, 这个其实不是很需要???
        self.drill_theta = (self.drill_theta + delta_theta) % (2 * np.pi)

        actual_theta = self.drill_theta

        # 添加动作噪声（论文III-B节）
        actual_theta += np.random.normal(0, 0.01 * abs(delta_theta)+1e-6)
        print("current_step: ", self.current_step, "actual_theta: ", actual_theta)

        # 更新微型钻头位置
        velocity = self.microdrill_speed * np.array([
            np.cos(actual_theta),
            np.sin(actual_theta)
        ])

        self.drill_pos += velocity * self.dt

        # 更新存在的障碍物位置（动态调整速度和替换越界障碍物）
        new_obstacles = []
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                # === 动态生成速度（公式3） ===
                speed_x = np.random.uniform( -self.microdrill_speed / 3,  self.microdrill_speed / 3)
                #speed_x = np.random.uniform(2, 10)
                speed_y = np.random.uniform( -self.microdrill_speed / 3,  self.microdrill_speed / 3)
                #speed_y = np.random.uniform(2, 10)
                obst['velocity'] = np.array([speed_x, speed_y])

                # 更新位置
                obst['position'] += obst['velocity'] * self.dt

                # === 检查边界并替换越界障碍物（论文III-A1节） ===
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
        terminated = self._check_collision()
        truncated = (self.current_step >= self.max_steps)

        # 计算奖励（公式6-10）
        reward = self._calculate_reward(velocity)

        self.current_step += 1

        return (
            self._get_obs(),
            reward,
            terminated,  # 明确终止条件
            truncated,  # 明确截断条件
            {}  # 必须返回五个值
        )

    def _check_collision(self):
        """碰撞检测"""
        # 检查边界碰撞
        if (self.drill_pos[0] < 0 or self.drill_pos[0] > self.env_width or self.drill_pos[1] < 0 or self.drill_pos[1] > self.env_height):
            return True

        # 检查障碍物碰撞
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                distance = np.linalg.norm(self.drill_pos - obst['position'])
                if distance <= (self.drill_radius + obst['radius']):
                    return True
            else:
                break
        return False


    def _calculate_reward(self, velocity):
        """实现论文III-A4节的奖励函数"""
        # 导航奖励（公式7）
        distance_to_goal = np.linalg.norm(self.drill_pos - self.goal_pos)
        scale = 25
        bn = 10 / ((abs(distance_to_goal)) / scale)  # ca=0.1

        # 障碍物惩罚（公式8）
        po = 0
        index_obs = 0
        d_safe = 45
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                dist = np.linalg.norm(self.drill_pos - obst['position']) - (obst['radius'] + self.drill_radius)
                # ??? 需不需要只考虑局部的呢?????
                if dist < d_safe:
                    po -= 5 * (1 / (abs(dist)) )
            else:
                break

        # 时间惩罚（公式9）
        pt = -0.01  # kt=0.01

        # 速度势能（公式10）
        pv = 0
        index_obs = 0
        for obst in self.obstacles:
            index_obs += 1
            if index_obs <= self.num_obstacles:
                relative_vel = velocity - obst['velocity']
                obstacle_to_drill = self.drill_pos - obst['position']
                unit_vector = obstacle_to_drill / (np.linalg.norm(obstacle_to_drill) + 1e-8)
                dot_product = np.dot(relative_vel, unit_vector)
                if dot_product < 0:
                    dist = np.linalg.norm(self.drill_pos - obst['position']) - (obst['radius'] + self.drill_radius)
                    # ??? 只考虑局部障碍物的影响
                    if dist < d_safe:
                        pv += 0.009 * dot_product
            else:
                break

        return bn + po + pt + pv



    def render(self):
        """实现可视化渲染"""
        if self.fig is None:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 15))
            self.ax.set_xlim(0, self.env_width)
            self.ax.set_ylim(0, self.env_height)
            self.ax.set_title("MicroDrill Navigation Simulation")

        # 清除上一帧内容
        self.ax.clear()
        self.ax.set_xlim(0, self.env_width)
        self.ax.set_ylim(0, self.env_height)

        # 绘制微型钻头
        drill_circle = plt.Circle(
            self.drill_pos,
            self.drill_radius,
            color='blue',
            alpha=0.8,
            label='Drill'
        )
        self.ax.add_patch(drill_circle)

        # 绘制方向箭头
        arrow_length = 5
        dx = arrow_length * np.cos(self.drill_theta)
        dy = arrow_length * np.sin(self.drill_theta)
        self.ax.arrow(
            self.drill_pos[0], self.drill_pos[1],
            dx, dy,
            head_width=3, head_length=3,
            fc='red', ec='red'
        )

        # 绘制目标区域
        goal_circle = plt.Circle(
            self.goal_pos,
            self.goal_radius,
            color='green',
            alpha=0.3,
            label='Goal'
        )

        self.ax.add_patch(goal_circle)

        # 绘制障碍物
        for obst in self.obstacles:
            obst_circle = plt.Circle(
                obst['position'],
                obst['radius'],
                color='orange',
                alpha=0.5,
                label='Obstacle'
            )
            self.ax.add_patch(obst_circle)
            # 绘制速度向量
            self.ax.arrow(
                obst['position'][0], obst['position'][1],
                obst['velocity'][0] * 5, obst['velocity'][1] * 5,  # 放大速度显示
                head_width=3, head_length=5,
                fc='black', ec='black'
            )

        # 添加图例和标注
        self.ax.legend(loc='upper right')
        self.ax.text(5, 290, f"Step: {self.current_step}/{self.max_steps}",
                     fontsize=10, bbox=dict(facecolor='white', alpha=0.8))

        print('current step: ', self.current_step)

        #plt.draw()
        # 强制刷新
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.15)  # 至少50ms


def run_simulation(model_path="sac_microdrill_raw.zip"):
    # 创建测试环境
    test_env = TestMicroDrillEnv(
        render_mode="human"
    )

    # 加载训练好的模型
    model = SAC.load(model_path)

    # 运行测试循环
    obs, _ = test_env.reset() #进行环境的设置/重置
    done = False
    total_reward = 0
    index_circle = 0
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = test_env.step(action)
        done = terminated or truncated
        total_reward += reward

        index_circle += 1
        print('current index_circle: ', index_circle)

        test_env.render()  # 渲染当前状态

        if done:
            if terminated:
                print("Episode terminated due to collision!")
            else:
                distance = np.linalg.norm(test_env.drill_pos - test_env.goal_pos)
                if distance <= test_env.goal_radius:
                    print("Reached goal successfully!")
                else:
                    print("Time out without reaching goal.")
            print(f"Total reward: {total_reward:.2f}")
            plt.ioff()
            plt.show()
        print('finish the current index_circle: ', index_circle)


if __name__ == "__main__":
    run_simulation()