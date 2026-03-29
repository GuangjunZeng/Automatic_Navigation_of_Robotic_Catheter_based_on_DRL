"""
3D Catheter Obstacle Avoidance - Matplotlib Version
基于Matplotlib的简单3D导管可视化（无需额外安装复杂依赖）

安装依赖: pip install matplotlib numpy scipy

适用于：快速测试、生成论文图片、无GPU环境
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.interpolate import CubicSpline
import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端以支持交互


class FlexibleCatheter3D:
    """3D柔性导管类"""
    
    def __init__(self, num_segments=12, segment_length=8.0):
        self.num_segments = num_segments
        self.segment_length = segment_length
        
        # 控制点初始化为直线
        self.control_points = np.zeros((num_segments + 1, 3))
        for i in range(num_segments + 1):
            self.control_points[i] = [i * segment_length, 0, 50]  # Z=50 作为初始高度
        
        # 速度
        self.velocity = 1.5
        self.angular_velocity = 0.08
        
        # 轨迹记录
        self.trajectory = []
    
    def get_tip(self):
        return self.control_points[-1].copy()
    
    def get_direction(self):
        if len(self.control_points) >= 2:
            direction = self.control_points[-1] - self.control_points[-2]
            return direction / (np.linalg.norm(direction) + 1e-6)
        return np.array([1, 0, 0])
    
    def get_smooth_curve(self, num_points=100):
        """生成平滑曲线"""
        t = np.linspace(0, 1, len(self.control_points))
        t_new = np.linspace(0, 1, num_points)
        
        curve = np.zeros((num_points, 3))
        for dim in range(3):
            cs = CubicSpline(t, self.control_points[:, dim])
            curve[:, dim] = cs(t_new)
        
        return curve
    
    def move_forward(self, speed=1.0):
        """向前移动"""
        direction = self.get_direction()
        delta = direction * self.velocity * speed
        self.control_points += delta
        
        # 记录轨迹
        self.trajectory.append(self.get_tip())
        if len(self.trajectory) > 300:
            self.trajectory.pop(0)
    
    def steer(self, pitch=0, yaw=0):
        """转向 (pitch: 上下, yaw: 左右)"""
        bend_segments = min(4, self.num_segments)
        
        for i in range(self.num_segments - bend_segments + 1, self.num_segments + 1):
            if i > 0:
                direction = self.control_points[i] - self.control_points[i-1]
                length = np.linalg.norm(direction)
                if length > 0:
                    direction = direction / length
                    
                    # 偏航旋转 (绕Z轴)
                    if yaw != 0:
                        cos_y, sin_y = np.cos(yaw * self.angular_velocity), np.sin(yaw * self.angular_velocity)
                        new_x = direction[0] * cos_y - direction[1] * sin_y
                        new_y = direction[0] * sin_y + direction[1] * cos_y
                        direction[0], direction[1] = new_x, new_y
                    
                    # 俯仰旋转 (绕Y轴)
                    if pitch != 0:
                        cos_p, sin_p = np.cos(pitch * self.angular_velocity), np.sin(pitch * self.angular_velocity)
                        new_x = direction[0] * cos_p + direction[2] * sin_p
                        new_z = -direction[0] * sin_p + direction[2] * cos_p
                        direction[0], direction[2] = new_x, new_z
                    
                    self.control_points[i] = self.control_points[i-1] + direction * self.segment_length


class Obstacle3D:
    """3D球形障碍物"""
    
    def __init__(self, position, radius=15, is_dynamic=False):
        self.position = np.array(position, dtype=float)
        self.radius = radius
        self.is_dynamic = is_dynamic
        self.velocity = np.zeros(3)
        
        if is_dynamic:
            self.velocity = np.random.uniform(-0.8, 0.8, 3)
            self.velocity[2] *= 0.3
    
    def update(self, bounds):
        if self.is_dynamic:
            self.position += self.velocity
            for i in range(3):
                if self.position[i] < bounds[i][0] or self.position[i] > bounds[i][1]:
                    self.velocity[i] *= -1
    
    def check_collision(self, point, margin=5):
        return np.linalg.norm(point - self.position) < (self.radius + margin)
    
    def get_surface(self, resolution=20):
        """生成球面网格用于绘制"""
        u = np.linspace(0, 2 * np.pi, resolution)
        v = np.linspace(0, np.pi, resolution)
        x = self.position[0] + self.radius * np.outer(np.cos(u), np.sin(v))
        y = self.position[1] + self.radius * np.outer(np.sin(u), np.sin(v))
        z = self.position[2] + self.radius * np.outer(np.ones(np.size(u)), np.cos(v))
        return x, y, z


class CatheterSimulation:
    """3D导管仿真主类"""
    
    def __init__(self):
        self.catheter = FlexibleCatheter3D(num_segments=10, segment_length=10)
        
        self.target = np.array([200, 0, 50])
        self.target_radius = 20
        
        self.obstacles = [
            Obstacle3D([60, 20, 50], radius=18),
            Obstacle3D([100, -15, 60], radius=15),
            Obstacle3D([140, 25, 45], radius=20),
            Obstacle3D([180, -10, 55], radius=12),
            Obstacle3D([80, 0, 70], radius=10, is_dynamic=True),
            Obstacle3D([160, 10, 40], radius=12, is_dynamic=True),
        ]
        
        self.bounds = [(-20, 250), (-80, 80), (10, 100)]
        self.sensor_range = 60
        
        self.auto_mode = True
        self.steps = 0
        
        # 创建图形
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.patch.set_facecolor('#1a1a2e')
        self.ax.set_facecolor('#16213e')
    
    def sense_obstacles(self):
        """感知障碍物距离"""
        tip = self.catheter.get_tip()
        tip_dir = self.catheter.get_direction()
        
        # 定义感知方向
        left_dir = np.array([-tip_dir[1], tip_dir[0], 0])
        left_dir = left_dir / (np.linalg.norm(left_dir) + 1e-6)
        
        directions = {
            'front': tip_dir,
            'left': left_dir,
            'right': -left_dir,
            'up': np.array([0, 0, 1]),
            'down': np.array([0, 0, -1]),
        }
        
        distances = {}
        for name, direction in directions.items():
            min_dist = self.sensor_range
            for obs in self.obstacles:
                # 简化的射线-球体检测
                to_obs = obs.position - tip
                proj = np.dot(to_obs, direction)
                if proj > 0:
                    closest = tip + direction * proj
                    dist_to_center = np.linalg.norm(closest - obs.position)
                    if dist_to_center < obs.radius:
                        surface_dist = proj - np.sqrt(obs.radius**2 - dist_to_center**2)
                        min_dist = min(min_dist, max(0, surface_dist))
            distances[name] = min_dist
        
        return distances
    
    def auto_control(self):
        """自动避障控制"""
        tip = self.catheter.get_tip()
        to_target = self.target - tip
        dist_to_target = np.linalg.norm(to_target)
        to_target_norm = to_target / (dist_to_target + 1e-6)
        
        distances = self.sense_obstacles()
        current_dir = self.catheter.get_direction()
        
        yaw, pitch, speed = 0, 0, 1.0
        
        if distances['front'] < 30:
            speed = 0.2
            yaw = 1 if distances['left'] < distances['right'] else -1
            pitch = 1 if distances['down'] < distances['up'] else -1
        elif distances['front'] < 50:
            speed = 0.5
            cross = np.cross(current_dir[:2], to_target_norm[:2])
            yaw = 1 if cross > 0.1 else (-1 if cross < -0.1 else 0)
            pitch = 1 if to_target_norm[2] > current_dir[2] + 0.1 else (-1 if to_target_norm[2] < current_dir[2] - 0.1 else 0)
        else:
            cross = np.cross(current_dir[:2], to_target_norm[:2])
            yaw = 1 if cross > 0.05 else (-1 if cross < -0.05 else 0)
            pitch = 1 if to_target_norm[2] > current_dir[2] + 0.05 else (-1 if to_target_norm[2] < current_dir[2] - 0.05 else 0)
        
        return pitch, yaw, speed
    
    def check_goal(self):
        tip = self.catheter.get_tip()
        return np.linalg.norm(tip - self.target) < self.target_radius
    
    def check_collision(self):
        curve = self.catheter.get_smooth_curve(30)
        for point in curve:
            for obs in self.obstacles:
                if obs.check_collision(point, margin=3):
                    return True
        return False
    
    def reset(self):
        self.catheter = FlexibleCatheter3D(num_segments=10, segment_length=10)
        for obs in self.obstacles:
            if obs.is_dynamic:
                obs.velocity = np.random.uniform(-0.8, 0.8, 3)
                obs.velocity[2] *= 0.3
        self.steps = 0
    
    def update(self, frame):
        """动画更新函数"""
        self.ax.clear()
        
        # 更新障碍物
        for obs in self.obstacles:
            obs.update(self.bounds)
        
        # 控制
        if self.auto_mode:
            pitch, yaw, speed = self.auto_control()
            self.catheter.steer(pitch=pitch, yaw=yaw)
            self.catheter.move_forward(speed)
        
        # 碰撞检测
        if self.check_collision():
            self.catheter.move_forward(-0.3)
        
        # 目标检测
        if self.check_goal():
            print(f"🎉 Goal reached in {self.steps} steps!")
            self.reset()
        
        self.steps += 1
        
        # ===== 绘制场景 =====
        
        # 设置背景和样式
        self.ax.set_facecolor('#16213e')
        
        # 绘制导管
        curve = self.catheter.get_smooth_curve(80)
        colors = plt.cm.magma(np.linspace(0.3, 0.9, len(curve)))
        for i in range(len(curve) - 1):
            self.ax.plot3D(
                curve[i:i+2, 0], curve[i:i+2, 1], curve[i:i+2, 2],
                color=colors[i], linewidth=3
            )
        
        # 绘制导管尖端
        tip = self.catheter.get_tip()
        self.ax.scatter(*tip, color='gold', s=100, marker='o', edgecolors='white', linewidth=2)
        
        # 绘制轨迹
        if len(self.catheter.trajectory) > 2:
            traj = np.array(self.catheter.trajectory)
            self.ax.plot3D(traj[:, 0], traj[:, 1], traj[:, 2], 
                          color='cyan', alpha=0.4, linewidth=1)
        
        # 绘制障碍物
        for obs in self.obstacles:
            x, y, z = obs.get_surface(15)
            color = 'red' if obs.is_dynamic else 'gray'
            alpha = 0.7 if obs.is_dynamic else 0.5
            self.ax.plot_surface(x, y, z, color=color, alpha=alpha)
        
        # 绘制目标
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = self.target[0] + self.target_radius * np.outer(np.cos(u), np.sin(v))
        y = self.target[1] + self.target_radius * np.outer(np.sin(u), np.sin(v))
        z = self.target[2] + self.target_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax.plot_surface(x, y, z, color='lime', alpha=0.4)
        
        # 设置坐标轴
        self.ax.set_xlim(self.bounds[0])
        self.ax.set_ylim(self.bounds[1])
        self.ax.set_zlim(self.bounds[2])
        self.ax.set_xlabel('X', color='white')
        self.ax.set_ylabel('Y', color='white')
        self.ax.set_zlabel('Z', color='white')
        
        # 设置标题
        mode_str = "AUTO" if self.auto_mode else "MANUAL"
        self.ax.set_title(
            f'3D Catheter Navigation | Mode: {mode_str} | Steps: {self.steps}',
            color='white', fontsize=14
        )
        
        # 设置刻度颜色
        self.ax.tick_params(colors='white')
        
        return []
    
    def run(self):
        """运行仿真"""
        print("=" * 50)
        print("3D Catheter Simulation (Matplotlib)")
        print("=" * 50)
        print("\n关闭窗口退出仿真")
        print("=" * 50)
        
        ani = FuncAnimation(
            self.fig, self.update,
            frames=None,
            interval=50,
            blit=False,
            cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    sim = CatheterSimulation()
    sim.run()
