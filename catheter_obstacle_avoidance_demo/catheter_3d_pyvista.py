"""
3D Catheter Obstacle Avoidance Simulation - PyVista Version
基于PyVista的3D柔性导管避障仿真演示

安装依赖: pip install pyvista numpy
"""

import numpy as np
import pyvista as pv
from scipy.interpolate import CubicSpline
import time


class Catheter3D:
    """3D柔性导管类"""
    
    def __init__(self, num_segments=15, segment_length=10.0):
        self.num_segments = num_segments
        self.segment_length = segment_length
        self.total_length = num_segments * segment_length
        
        # 导管控制点 (x, y, z)
        self.control_points = np.zeros((num_segments + 1, 3))
        self._initialize_straight()
        
        # 导管尖端方向
        self.tip_direction = np.array([1.0, 0.0, 0.0])
        
        # 物理参数
        self.stiffness = 0.8  # 刚度 (0-1, 越高越硬)
        self.damping = 0.3    # 阻尼
        
        # 速度
        self.velocity = 2.0
        self.angular_velocity = 0.05
        
        # 轨迹
        self.trajectory = []
    
    def _initialize_straight(self):
        """初始化为直线导管"""
        for i in range(self.num_segments + 1):
            self.control_points[i] = [i * self.segment_length, 0, 0]
    
    def get_tip_position(self):
        """获取导管尖端位置"""
        return self.control_points[-1].copy()
    
    def get_base_position(self):
        """获取导管基座位置"""
        return self.control_points[0].copy()
    
    def get_smooth_curve(self, num_points=100):
        """使用样条插值生成平滑的导管曲线"""
        t = np.linspace(0, 1, len(self.control_points))
        t_new = np.linspace(0, 1, num_points)
        
        # 对x, y, z分别进行样条插值
        curve = np.zeros((num_points, 3))
        for dim in range(3):
            cs = CubicSpline(t, self.control_points[:, dim])
            curve[:, dim] = cs(t_new)
        
        return curve
    
    def move_forward(self, speed=1.0):
        """向前推进导管"""
        # 计算尖端方向
        if len(self.control_points) >= 2:
            self.tip_direction = self.control_points[-1] - self.control_points[-2]
            self.tip_direction = self.tip_direction / (np.linalg.norm(self.tip_direction) + 1e-6)
        
        # 移动所有控制点
        delta = self.tip_direction * self.velocity * speed
        self.control_points += delta
        
        # 记录轨迹
        self.trajectory.append(self.get_tip_position())
        if len(self.trajectory) > 500:
            self.trajectory.pop(0)
    
    def steer(self, pitch=0, yaw=0):
        """
        转向控制
        pitch: 俯仰角变化 (上下)
        yaw: 偏航角变化 (左右)
        """
        # 只弯曲最后几个节段
        bend_segments = min(5, self.num_segments)
        
        for i in range(self.num_segments - bend_segments + 1, self.num_segments + 1):
            # 计算当前方向
            if i > 0:
                direction = self.control_points[i] - self.control_points[i-1]
                length = np.linalg.norm(direction)
                if length > 0:
                    direction = direction / length
                    
                    # 应用偏航 (绕Z轴旋转)
                    if yaw != 0:
                        cos_yaw = np.cos(yaw * self.angular_velocity)
                        sin_yaw = np.sin(yaw * self.angular_velocity)
                        new_x = direction[0] * cos_yaw - direction[1] * sin_yaw
                        new_y = direction[0] * sin_yaw + direction[1] * cos_yaw
                        direction[0] = new_x
                        direction[1] = new_y
                    
                    # 应用俯仰 (绕Y轴旋转)
                    if pitch != 0:
                        cos_pitch = np.cos(pitch * self.angular_velocity)
                        sin_pitch = np.sin(pitch * self.angular_velocity)
                        new_x = direction[0] * cos_pitch + direction[2] * sin_pitch
                        new_z = -direction[0] * sin_pitch + direction[2] * cos_pitch
                        direction[0] = new_x
                        direction[2] = new_z
                    
                    # 更新控制点位置
                    self.control_points[i] = self.control_points[i-1] + direction * self.segment_length
    
    def apply_constraint(self):
        """应用长度约束，保持各段长度不变"""
        for i in range(1, len(self.control_points)):
            direction = self.control_points[i] - self.control_points[i-1]
            length = np.linalg.norm(direction)
            if length > 0:
                direction = direction / length
                self.control_points[i] = self.control_points[i-1] + direction * self.segment_length


class Obstacle3D:
    """3D障碍物类"""
    
    def __init__(self, position, radius=15.0, is_dynamic=False):
        self.position = np.array(position, dtype=float)
        self.radius = radius
        self.is_dynamic = is_dynamic
        self.velocity = np.zeros(3)
        
        if is_dynamic:
            self.velocity = np.random.uniform(-1, 1, 3)
            self.velocity[2] *= 0.3  # Z方向速度较小
    
    def update(self, bounds):
        """更新动态障碍物"""
        if self.is_dynamic:
            self.position += self.velocity
            
            # 边界反弹
            for i in range(3):
                if self.position[i] < bounds[i][0] or self.position[i] > bounds[i][1]:
                    self.velocity[i] *= -1
    
    def check_collision(self, point, margin=5.0):
        """检查点是否与障碍物碰撞"""
        dist = np.linalg.norm(point - self.position)
        return dist < (self.radius + margin)
    
    def get_mesh(self):
        """获取障碍物的网格用于渲染"""
        return pv.Sphere(radius=self.radius, center=self.position)


class CatheterSimulation3D:
    """3D导管仿真主类"""
    
    def __init__(self):
        # 创建导管
        self.catheter = Catheter3D(num_segments=12, segment_length=12.0)
        
        # 目标位置
        self.target = np.array([250.0, 0.0, 0.0])
        self.target_radius = 20.0
        
        # 创建障碍物
        self.obstacles = self._create_obstacles()
        
        # 边界
        self.bounds = [(-50, 300), (-100, 100), (-80, 80)]
        
        # 仿真状态
        self.running = True
        self.auto_mode = True
        self.steps = 0
        self.collisions = 0
        
        # 传感器范围
        self.sensor_range = 80.0
    
    def _create_obstacles(self):
        """创建障碍物"""
        obstacles = [
            # 静态障碍物
            Obstacle3D([80, 30, 0], radius=20),
            Obstacle3D([120, -20, 15], radius=18),
            Obstacle3D([160, 25, -10], radius=22),
            Obstacle3D([200, -30, 20], radius=15),
            Obstacle3D([140, 0, -25], radius=16),
            
            # 动态障碍物
            Obstacle3D([100, 0, 30], radius=12, is_dynamic=True),
            Obstacle3D([180, 20, -15], radius=14, is_dynamic=True),
        ]
        return obstacles
    
    def check_collision(self):
        """检查导管是否碰撞"""
        curve = self.catheter.get_smooth_curve(50)
        for point in curve:
            for obs in self.obstacles:
                if obs.check_collision(point, margin=3):
                    return True
        return False
    
    def check_goal(self):
        """检查是否到达目标"""
        tip = self.catheter.get_tip_position()
        dist = np.linalg.norm(tip - self.target)
        return dist < self.target_radius
    
    def sense_obstacles(self):
        """感知周围障碍物，返回各方向的距离"""
        tip = self.catheter.get_tip_position()
        tip_dir = self.catheter.tip_direction
        
        # 6个方向: 前、左、右、上、下、后
        directions = {
            'front': tip_dir,
            'left': np.array([-tip_dir[1], tip_dir[0], 0]),
            'right': np.array([tip_dir[1], -tip_dir[0], 0]),
            'up': np.array([0, 0, 1]),
            'down': np.array([0, 0, -1]),
        }
        
        distances = {}
        for name, direction in directions.items():
            direction = direction / (np.linalg.norm(direction) + 1e-6)
            min_dist = self.sensor_range
            
            for obs in self.obstacles:
                # 射线-球体相交检测
                oc = tip - obs.position
                a = np.dot(direction, direction)
                b = 2.0 * np.dot(oc, direction)
                c = np.dot(oc, oc) - obs.radius ** 2
                discriminant = b * b - 4 * a * c
                
                if discriminant > 0:
                    t = (-b - np.sqrt(discriminant)) / (2.0 * a)
                    if 0 < t < min_dist:
                        min_dist = t
            
            distances[name] = min_dist
        
        return distances
    
    def auto_control(self):
        """自动避障控制"""
        tip = self.catheter.get_tip_position()
        
        # 到目标的方向
        to_target = self.target - tip
        dist_to_target = np.linalg.norm(to_target)
        to_target_normalized = to_target / (dist_to_target + 1e-6)
        
        # 感知障碍物
        distances = self.sense_obstacles()
        
        # 计算期望的偏航和俯仰
        yaw = 0
        pitch = 0
        speed = 1.0
        
        # 基于障碍物距离的避障
        if distances['front'] < 40:
            speed = 0.2
            if distances['left'] > distances['right']:
                yaw = -1
            else:
                yaw = 1
            if distances['up'] > distances['down']:
                pitch = 1
            else:
                pitch = -1
        elif distances['front'] < 60:
            speed = 0.5
            # 朝目标微调
            current_dir = self.catheter.tip_direction
            
            # 计算偏航调整
            cross_z = current_dir[0] * to_target_normalized[1] - current_dir[1] * to_target_normalized[0]
            if cross_z > 0.1:
                yaw = 1
            elif cross_z < -0.1:
                yaw = -1
            
            # 计算俯仰调整
            if to_target_normalized[2] > current_dir[2] + 0.1:
                pitch = 1
            elif to_target_normalized[2] < current_dir[2] - 0.1:
                pitch = -1
        else:
            # 自由移动，朝向目标
            current_dir = self.catheter.tip_direction
            
            cross_z = current_dir[0] * to_target_normalized[1] - current_dir[1] * to_target_normalized[0]
            if cross_z > 0.05:
                yaw = 1
            elif cross_z < -0.05:
                yaw = -1
            
            if to_target_normalized[2] > current_dir[2] + 0.05:
                pitch = 1
            elif to_target_normalized[2] < current_dir[2] - 0.05:
                pitch = -1
        
        return pitch, yaw, speed
    
    def reset(self):
        """重置仿真"""
        self.catheter = Catheter3D(num_segments=12, segment_length=12.0)
        self.obstacles = self._create_obstacles()
        self.steps = 0
        self.collisions = 0
    
    def run(self):
        """运行仿真"""
        # 创建PyVista绘图器
        plotter = pv.Plotter()
        plotter.set_background('black')
        
        # 添加文字说明
        plotter.add_text(
            "3D Catheter Navigation\n"
            "Controls: Arrow Keys (manual), Space (toggle auto), R (reset), Q (quit)",
            position='upper_left',
            font_size=10,
            color='white'
        )
        
        # 添加坐标轴
        plotter.add_axes()
        
        # 初始化演员
        catheter_actor = None
        trajectory_actor = None
        obstacle_actors = []
        target_actor = None
        
        def update_scene():
            nonlocal catheter_actor, trajectory_actor, obstacle_actors, target_actor
            
            # 清除之前的演员
            if catheter_actor is not None:
                plotter.remove_actor(catheter_actor)
            if trajectory_actor is not None:
                plotter.remove_actor(trajectory_actor)
            for actor in obstacle_actors:
                plotter.remove_actor(actor)
            obstacle_actors.clear()
            if target_actor is not None:
                plotter.remove_actor(target_actor)
            
            # 绘制导管
            curve = self.catheter.get_smooth_curve(100)
            catheter_mesh = pv.Spline(curve, n_points=100)
            catheter_actor = plotter.add_mesh(
                catheter_mesh.tube(radius=3),
                color='magenta',
                smooth_shading=True
            )
            
            # 绘制导管尖端
            tip = self.catheter.get_tip_position()
            tip_sphere = pv.Sphere(radius=5, center=tip)
            plotter.add_mesh(tip_sphere, color='gold')
            
            # 绘制轨迹
            if len(self.catheter.trajectory) > 2:
                traj = np.array(self.catheter.trajectory)
                traj_line = pv.Spline(traj, n_points=len(traj))
                trajectory_actor = plotter.add_mesh(
                    traj_line.tube(radius=1),
                    color='cyan',
                    opacity=0.5
                )
            
            # 绘制障碍物
            for obs in self.obstacles:
                color = 'red' if obs.is_dynamic else 'gray'
                actor = plotter.add_mesh(obs.get_mesh(), color=color, opacity=0.7)
                obstacle_actors.append(actor)
            
            # 绘制目标
            target_mesh = pv.Sphere(radius=self.target_radius, center=self.target)
            target_actor = plotter.add_mesh(target_mesh, color='green', opacity=0.5)
            
            # 绘制边界框
            bounds_box = pv.Box(bounds=(
                self.bounds[0][0], self.bounds[0][1],
                self.bounds[1][0], self.bounds[1][1],
                self.bounds[2][0], self.bounds[2][1]
            ))
            plotter.add_mesh(bounds_box, style='wireframe', color='white', opacity=0.2)
        
        def callback(caller, event):
            if not self.running:
                return
            
            # 更新障碍物
            for obs in self.obstacles:
                obs.update(self.bounds)
            
            # 自动控制
            if self.auto_mode:
                pitch, yaw, speed = self.auto_control()
                self.catheter.steer(pitch=pitch, yaw=yaw)
                self.catheter.move_forward(speed)
                self.catheter.apply_constraint()
            
            # 碰撞检测
            if self.check_collision():
                self.collisions += 1
                self.catheter.move_forward(-0.5)
            
            # 目标检测
            if self.check_goal():
                print("🎉 Goal Reached!")
                self.reset()
            
            self.steps += 1
            update_scene()
        
        # 键盘控制
        def key_callback(key):
            if key == 'space':
                self.auto_mode = not self.auto_mode
                print(f"Mode: {'AUTO' if self.auto_mode else 'MANUAL'}")
            elif key == 'r':
                self.reset()
            elif key == 'q':
                self.running = False
                plotter.close()
            elif not self.auto_mode:
                if key == 'Up':
                    self.catheter.move_forward(1.0)
                elif key == 'Down':
                    self.catheter.move_forward(-0.5)
                elif key == 'Left':
                    self.catheter.steer(yaw=-1)
                elif key == 'Right':
                    self.catheter.steer(yaw=1)
                elif key == 'w':
                    self.catheter.steer(pitch=1)
                elif key == 's':
                    self.catheter.steer(pitch=-1)
                self.catheter.apply_constraint()
        
        plotter.add_key_event('space', lambda: key_callback('space'))
        plotter.add_key_event('r', lambda: key_callback('r'))
        plotter.add_key_event('q', lambda: key_callback('q'))
        plotter.add_key_event('Up', lambda: key_callback('Up'))
        plotter.add_key_event('Down', lambda: key_callback('Down'))
        plotter.add_key_event('Left', lambda: key_callback('Left'))
        plotter.add_key_event('Right', lambda: key_callback('Right'))
        plotter.add_key_event('w', lambda: key_callback('w'))
        plotter.add_key_event('s', lambda: key_callback('s'))
        
        # 初始场景
        update_scene()
        
        # 设置相机
        plotter.camera_position = [(150, -200, 150), (150, 0, 0), (0, 0, 1)]
        
        # 添加定时器回调
        plotter.add_callback(callback, interval=50)
        
        # 显示
        plotter.show()


if __name__ == "__main__":
    print("=" * 50)
    print("3D Catheter Obstacle Avoidance Simulation")
    print("=" * 50)
    print("\nControls:")
    print("  SPACE - Toggle Auto/Manual mode")
    print("  Arrow Keys - Move forward/backward, turn left/right")
    print("  W/S - Pitch up/down")
    print("  R - Reset simulation")
    print("  Q - Quit")
    print("\n" + "=" * 50)
    
    sim = CatheterSimulation3D()
    sim.run()
