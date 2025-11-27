# 🔬 3D Catheter Obstacle Avoidance Simulation

三维柔性导管避障仿真Demo，提供多个版本供选择。

## 📁 文件说明

| 文件 | 描述 | 适用场景 |
|------|------|----------|
| `catheter_3d_pyvista.py` | PyVista 3D可视化版本 | 快速演示、论文图片 |
| `catheter_3d_pybullet.py` | PyBullet 物理仿真版本 | 真实物理、RL训练 |
| `catheter_sim.py` | 2D PyGame版本 | 简单测试、快速迭代 |

## 🚀 快速开始

### 1. 安装依赖

```bash
cd catheter_obstacle_avoidance_demo
pip install -r requirements.txt
```

### 2. 运行PyVista版本 (推荐)

```bash
python catheter_3d_pyvista.py
```

**控制方式:**
- `SPACE` - 切换自动/手动模式
- `↑/↓` - 前进/后退
- `←/→` - 左转/右转
- `W/S` - 俯仰上/下
- `R` - 重置
- `Q` - 退出

### 3. 运行PyBullet版本

```bash
python catheter_3d_pybullet.py
```

**控制方式:**
- GUI滑块调节参数
- 鼠标旋转/缩放视角
- `R` - 重置
- `Q` - 退出

## 🎯 功能特性

### PyVista版本
- ✅ 柔性导管建模（样条曲线插值）
- ✅ 3D可视化渲染
- ✅ 静态/动态障碍物
- ✅ 简单避障算法
- ✅ 轨迹记录显示
- ✅ 传感器模拟

### PyBullet版本
- ✅ 真实物理引擎
- ✅ 碰撞检测
- ✅ 多段刚体链接导管
- ✅ 射线传感器
- ✅ 重力模拟
- ⬜ 软体（Soft Body）支持 (可扩展)

## 🔧 自定义配置

### 修改导管参数
```python
# PyVista版本
catheter = Catheter3D(
    num_segments=15,      # 节段数量
    segment_length=10.0   # 每段长度
)

# PyBullet版本
catheter = CatheterPyBullet(
    base_position=[0, 0, 0.5],
    num_segments=12,
    segment_length=0.06
)
```

### 修改障碍物
```python
obstacles = [
    Obstacle3D([x, y, z], radius=20, is_dynamic=False),
    Obstacle3D([x, y, z], radius=15, is_dynamic=True),
]
```

## 📚 进阶开发

### 1. 集成强化学习

```python
import gymnasium as gym
from stable_baselines3 import PPO

# 将仿真环境封装为Gym环境
class CatheterEnv(gym.Env):
    def __init__(self):
        self.sim = CatheterSimulation3D()
        self.observation_space = ...
        self.action_space = ...
    
    def step(self, action):
        # 执行动作，返回观测、奖励等
        pass
    
    def reset(self):
        self.sim.reset()
        return observation, info

# 训练
env = CatheterEnv()
model = PPO("MlpPolicy", env)
model.learn(total_timesteps=100000)
```

### 2. 使用MuJoCo (更精确的物理)

如需更精确的柔性体仿真，推荐使用MuJoCo:

```bash
pip install mujoco
```

MuJoCo支持:
- Tendon（腱）模型
- Cable（电缆）模型
- 更精确的接触力学

## 🔗 相关资源

- [PyVista文档](https://docs.pyvista.org/)
- [PyBullet快速入门](https://pybullet.org/wordpress/)
- [MuJoCo文档](https://mujoco.readthedocs.io/)
- [CoppeliaSim](https://www.coppeliarobotics.com/)

## 📝 TODO

- [ ] 添加MuJoCo版本
- [ ] 添加更复杂的避障算法 (RRT*, A*)
- [ ] 添加强化学习训练脚本
- [ ] 支持血管环境建模
- [ ] 添加力反馈模拟
