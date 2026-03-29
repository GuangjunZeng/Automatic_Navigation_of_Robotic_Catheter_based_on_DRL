"""
Catheter Obstacle Avoidance Simulation Demo
基于pygame的2D柔性导管避障仿真
"""

import numpy as np
import pygame
import math
from collections import deque

# ============== 颜色定义 ==============
COLORS = {
    'background': (15, 23, 42),      # 深蓝色背景
    'catheter': (236, 72, 153),       # 粉紫色导管
    'catheter_tip': (251, 191, 36),   # 金色导管尖端
    'obstacle_static': (71, 85, 105), # 静态障碍物
    'obstacle_dynamic': (239, 68, 68),# 动态障碍物（红色）
    'target': (34, 197, 94),          # 目标点（绿色）
    'path': (99, 102, 241),           # 路径轨迹
    'grid': (30, 41, 59),             # 网格线
    'text': (226, 232, 240),          # 文字
    'sensor': (56, 189, 248),         # 传感器射线
}


class CatheterSegment:
    """导管节段类 - 模拟导管的一个关节"""
    def __init__(self, x, y, angle, length=20):
        self.x = x
        self.y = y
        self.angle = angle  # 弧度
        self.length = length
    
    def get_end_point(self):
        """获取该节段的末端点"""
        end_x = self.x + self.length * math.cos(self.angle)
        end_y = self.y + self.length * math.sin(self.angle)
        return end_x, end_y


class FlexibleCatheter:
    """柔性导管类 - 由多个节段组成"""
    def __init__(self, base_x, base_y, num_segments=8, segment_length=25):
        self.base_x = base_x
        self.base_y = base_y
        self.num_segments = num_segments
        self.segment_length = segment_length
        self.segments = []
        self.tip_radius = 8  # 导管尖端半径（用于碰撞检测）
        
        # 初始化各节段（初始为直线）
        angle = 0  # 初始朝右
        x, y = base_x, base_y
        for i in range(num_segments):
            seg = CatheterSegment(x, y, angle, segment_length)
            self.segments.append(seg)
            x, y = seg.get_end_point()
        
        # 导管速度参数
        self.velocity = 2.0  # 基础移动速度
        self.angular_velocity = 0.05  # 角速度
        
        # 轨迹记录
        self.trajectory = deque(maxlen=200)
    
    def get_tip_position(self):
        """获取导管尖端位置"""
        if self.segments:
            return self.segments[-1].get_end_point()
        return self.base_x, self.base_y
    
    def get_all_points(self):
        """获取导管所有关节点（用于绘制）"""
        points = [(self.segments[0].x, self.segments[0].y)]
        for seg in self.segments:
            points.append(seg.get_end_point())
        return points
    
    def move_forward(self, speed_factor=1.0):
        """向前推进导管"""
        if not self.segments:
            return
        
        # 获取最后一段的方向，向该方向移动
        last_seg = self.segments[-1]
        dx = self.velocity * speed_factor * math.cos(last_seg.angle)
        dy = self.velocity * speed_factor * math.sin(last_seg.angle)
        
        # 移动所有节段
        for seg in self.segments:
            seg.x += dx
            seg.y += dy
        
        self.base_x += dx
        self.base_y += dy
        
        # 记录轨迹
        tip = self.get_tip_position()
        self.trajectory.append(tip)
    
    def steer(self, direction):
        """转向控制 - 改变尖端方向
        direction: -1 (左转), 0 (直行), 1 (右转)
        """
        if not self.segments:
            return
        
        # 只改变最后几个节段的角度（模拟柔性弯曲）
        bend_segments = min(3, len(self.segments))  # 弯曲的节段数
        delta_angle = direction * self.angular_velocity
        
        for i in range(len(self.segments) - bend_segments, len(self.segments)):
            self.segments[i].angle += delta_angle
        
        # 更新后续节段的位置（从弯曲点开始重新计算）
        self._update_segment_positions()
    
    def _update_segment_positions(self):
        """更新节段位置（保持连续性）"""
        for i in range(1, len(self.segments)):
            prev_seg = self.segments[i-1]
            prev_end_x, prev_end_y = prev_seg.get_end_point()
            self.segments[i].x = prev_end_x
            self.segments[i].y = prev_end_y
    
    def avoid_obstacle(self, obstacles, sensor_range=100):
        """简单的避障逻辑 - 基于传感器检测"""
        tip_x, tip_y = self.get_tip_position()
        last_seg = self.segments[-1]
        
        # 检测前方三个方向的障碍物
        angles = [last_seg.angle - 0.5, last_seg.angle, last_seg.angle + 0.5]
        distances = []
        
        for angle in angles:
            min_dist = sensor_range
            for obs in obstacles:
                # 计算射线与障碍物的距离
                dist = self._ray_obstacle_distance(tip_x, tip_y, angle, obs, sensor_range)
                min_dist = min(min_dist, dist)
            distances.append(min_dist)
        
        return distances
    
    def _ray_obstacle_distance(self, x, y, angle, obstacle, max_dist):
        """计算射线到障碍物的距离"""
        for d in range(0, int(max_dist), 5):
            check_x = x + d * math.cos(angle)
            check_y = y + d * math.sin(angle)
            if obstacle.contains_point(check_x, check_y):
                return d
        return max_dist


class Obstacle:
    """障碍物基类"""
    def __init__(self, x, y, is_dynamic=False):
        self.x = x
        self.y = y
        self.is_dynamic = is_dynamic
        self.vx = 0
        self.vy = 0
        if is_dynamic:
            self.vx = np.random.uniform(-1.5, 1.5)
            self.vy = np.random.uniform(-1.5, 1.5)
    
    def update(self, bounds):
        """更新动态障碍物位置"""
        if self.is_dynamic:
            self.x += self.vx
            self.y += self.vy
            
            # 边界反弹
            if self.x < bounds[0] or self.x > bounds[2]:
                self.vx *= -1
            if self.y < bounds[1] or self.y > bounds[3]:
                self.vy *= -1
    
    def contains_point(self, px, py):
        """检查点是否在障碍物内"""
        raise NotImplementedError
    
    def draw(self, surface):
        """绘制障碍物"""
        raise NotImplementedError


class CircleObstacle(Obstacle):
    """圆形障碍物"""
    def __init__(self, x, y, radius, is_dynamic=False):
        super().__init__(x, y, is_dynamic)
        self.radius = radius
    
    def contains_point(self, px, py):
        dist = math.sqrt((px - self.x)**2 + (py - self.y)**2)
        return dist < self.radius
    
    def draw(self, surface):
        color = COLORS['obstacle_dynamic'] if self.is_dynamic else COLORS['obstacle_static']
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), self.radius)
        # 添加高光效果
        highlight_pos = (int(self.x - self.radius*0.3), int(self.y - self.radius*0.3))
        pygame.draw.circle(surface, (255, 255, 255, 100), highlight_pos, self.radius//3)


class RectObstacle(Obstacle):
    """矩形障碍物"""
    def __init__(self, x, y, width, height, is_dynamic=False):
        super().__init__(x, y, is_dynamic)
        self.width = width
        self.height = height
    
    def contains_point(self, px, py):
        return (self.x <= px <= self.x + self.width and 
                self.y <= py <= self.y + self.height)
    
    def draw(self, surface):
        color = COLORS['obstacle_dynamic'] if self.is_dynamic else COLORS['obstacle_static']
        rect = pygame.Rect(int(self.x), int(self.y), self.width, self.height)
        pygame.draw.rect(surface, color, rect, border_radius=5)


class CatheterSimulation:
    """导管仿真主类"""
    def __init__(self, width=900, height=700):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("🔬 Catheter Obstacle Avoidance Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.font_large = pygame.font.Font(None, 36)
        
        # 初始化导管
        self.catheter = FlexibleCatheter(100, height//2, num_segments=10, segment_length=20)
        
        # 目标点
        self.target = (width - 100, height//2)
        self.target_radius = 30
        
        # 初始化障碍物
        self.obstacles = self._create_obstacles()
        
        # 传感器范围
        self.sensor_range = 120
        
        # 控制模式
        self.auto_mode = True  # True: 自动避障, False: 手动控制
        
        # 统计信息
        self.steps = 0
        self.collisions = 0
        self.running = True
    
    def _create_obstacles(self):
        """创建障碍物布局"""
        obstacles = []
        
        # 静态障碍物 - 形成一个需要绕过的迷宫式布局
        static_obstacles = [
            # 中间区域障碍物
            CircleObstacle(300, 250, 40),
            CircleObstacle(400, 450, 35),
            CircleObstacle(550, 300, 45),
            CircleObstacle(650, 500, 30),
            
            # 矩形障碍物
            RectObstacle(200, 400, 80, 30),
            RectObstacle(450, 150, 30, 100),
            RectObstacle(600, 350, 100, 25),
            RectObstacle(350, 550, 120, 25),
        ]
        obstacles.extend(static_obstacles)
        
        # 动态障碍物
        dynamic_obstacles = [
            CircleObstacle(500, 350, 25, is_dynamic=True),
            CircleObstacle(300, 500, 20, is_dynamic=True),
            CircleObstacle(700, 200, 22, is_dynamic=True),
        ]
        obstacles.extend(dynamic_obstacles)
        
        return obstacles
    
    def check_collision(self):
        """检查导管是否与障碍物碰撞"""
        points = self.catheter.get_all_points()
        for point in points:
            for obs in self.obstacles:
                if obs.contains_point(point[0], point[1]):
                    return True
        return False
    
    def check_goal_reached(self):
        """检查是否到达目标"""
        tip = self.catheter.get_tip_position()
        dist = math.sqrt((tip[0] - self.target[0])**2 + (tip[1] - self.target[1])**2)
        return dist < self.target_radius
    
    def auto_control(self):
        """自动避障控制逻辑"""
        tip_x, tip_y = self.catheter.get_tip_position()
        last_seg = self.catheter.segments[-1]
        
        # 计算到目标的角度
        angle_to_target = math.atan2(self.target[1] - tip_y, self.target[0] - tip_x)
        
        # 获取传感器数据
        sensor_data = self.catheter.avoid_obstacle(self.obstacles, self.sensor_range)
        left_dist, front_dist, right_dist = sensor_data
        
        # 决策逻辑
        steer = 0
        speed = 1.0
        
        # 如果前方有障碍物
        if front_dist < 60:
            speed = 0.3  # 减速
            # 选择更空旷的方向
            if left_dist > right_dist:
                steer = -1
            else:
                steer = 1
        elif front_dist < 100:
            speed = 0.6
            # 朝目标方向微调
            angle_diff = angle_to_target - last_seg.angle
            # 归一化角度差
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if angle_diff > 0.1:
                steer = 1
            elif angle_diff < -0.1:
                steer = -1
        else:
            # 前方安全，朝目标移动
            angle_diff = angle_to_target - last_seg.angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if angle_diff > 0.05:
                steer = 1
            elif angle_diff < -0.05:
                steer = -1
        
        return steer, speed
    
    def draw_grid(self):
        """绘制背景网格"""
        for x in range(0, self.width, 50):
            pygame.draw.line(self.screen, COLORS['grid'], (x, 0), (x, self.height))
        for y in range(0, self.height, 50):
            pygame.draw.line(self.screen, COLORS['grid'], (0, y), (self.width, y))
    
    def draw_sensors(self):
        """绘制传感器射线"""
        tip_x, tip_y = self.catheter.get_tip_position()
        last_seg = self.catheter.segments[-1]
        
        angles = [last_seg.angle - 0.5, last_seg.angle, last_seg.angle + 0.5]
        sensor_data = self.catheter.avoid_obstacle(self.obstacles, self.sensor_range)
        
        for angle, dist in zip(angles, sensor_data):
            end_x = tip_x + dist * math.cos(angle)
            end_y = tip_y + dist * math.sin(angle)
            
            # 根据距离选择颜色
            if dist < 50:
                color = (239, 68, 68)  # 红色 - 危险
            elif dist < 80:
                color = (251, 191, 36)  # 黄色 - 警告
            else:
                color = COLORS['sensor']  # 蓝色 - 安全
            
            pygame.draw.line(self.screen, color, (int(tip_x), int(tip_y)), 
                           (int(end_x), int(end_y)), 2)
            pygame.draw.circle(self.screen, color, (int(end_x), int(end_y)), 4)
    
    def draw_catheter(self):
        """绘制导管"""
        points = self.catheter.get_all_points()
        
        # 绘制轨迹
        if len(self.catheter.trajectory) > 1:
            traj_points = list(self.catheter.trajectory)
            for i in range(1, len(traj_points)):
                alpha = int(255 * i / len(traj_points))
                color = (COLORS['path'][0], COLORS['path'][1], COLORS['path'][2])
                pygame.draw.line(self.screen, color, 
                               (int(traj_points[i-1][0]), int(traj_points[i-1][1])),
                               (int(traj_points[i][0]), int(traj_points[i][1])), 2)
        
        # 绘制导管主体 - 渐变粗细
        if len(points) > 1:
            for i in range(len(points) - 1):
                # 从粗到细
                thickness = max(3, 10 - i * 0.8)
                # 颜色渐变
                ratio = i / len(points)
                color = (
                    int(COLORS['catheter'][0] * (1 - ratio * 0.3)),
                    int(COLORS['catheter'][1] * (1 - ratio * 0.3)),
                    int(COLORS['catheter'][2] * (1 - ratio * 0.3))
                )
                pygame.draw.line(self.screen, color,
                               (int(points[i][0]), int(points[i][1])),
                               (int(points[i+1][0]), int(points[i+1][1])),
                               int(thickness))
        
        # 绘制导管尖端
        tip = self.catheter.get_tip_position()
        pygame.draw.circle(self.screen, COLORS['catheter_tip'], 
                          (int(tip[0]), int(tip[1])), 6)
        
        # 绘制关节点
        for i, point in enumerate(points):
            radius = max(2, 4 - i * 0.3)
            pygame.draw.circle(self.screen, (255, 255, 255), 
                             (int(point[0]), int(point[1])), int(radius))
    
    def draw_target(self):
        """绘制目标点"""
        # 外圈动画
        pulse = abs(math.sin(pygame.time.get_ticks() * 0.005)) * 10
        pygame.draw.circle(self.screen, COLORS['target'], 
                          self.target, int(self.target_radius + pulse), 3)
        # 内圈
        pygame.draw.circle(self.screen, COLORS['target'], 
                          self.target, self.target_radius - 10)
        # 中心点
        pygame.draw.circle(self.screen, (255, 255, 255), 
                          self.target, 5)
    
    def draw_ui(self):
        """绘制用户界面"""
        # 标题
        title = self.font_large.render("Catheter Navigation Demo", True, COLORS['text'])
        self.screen.blit(title, (10, 10))
        
        # 模式显示
        mode_text = "AUTO" if self.auto_mode else "MANUAL"
        mode_color = (34, 197, 94) if self.auto_mode else (251, 191, 36)
        mode = self.font.render(f"Mode: {mode_text}", True, mode_color)
        self.screen.blit(mode, (10, 50))
        
        # 统计信息
        stats = [
            f"Steps: {self.steps}",
            f"Collisions: {self.collisions}",
        ]
        for i, stat in enumerate(stats):
            text = self.font.render(stat, True, COLORS['text'])
            self.screen.blit(text, (10, 80 + i * 25))
        
        # 控制说明
        instructions = [
            "Controls:",
            "SPACE - Toggle Auto/Manual",
            "↑/↓ - Forward/Backward",
            "←/→ - Steer Left/Right",
            "R - Reset",
            "ESC - Quit"
        ]
        for i, inst in enumerate(instructions):
            text = self.font.render(inst, True, COLORS['text'])
            self.screen.blit(text, (self.width - 200, 10 + i * 25))
    
    def reset(self):
        """重置仿真"""
        self.catheter = FlexibleCatheter(100, self.height//2, num_segments=10, segment_length=20)
        self.obstacles = self._create_obstacles()
        self.steps = 0
        self.collisions = 0
    
    def run(self):
        """主循环"""
        while self.running:
            # 事件处理
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        self.auto_mode = not self.auto_mode
                    elif event.key == pygame.K_r:
                        self.reset()
            
            # 更新障碍物
            bounds = (50, 50, self.width - 50, self.height - 50)
            for obs in self.obstacles:
                obs.update(bounds)
            
            # 控制逻辑
            if self.auto_mode:
                steer, speed = self.auto_control()
            else:
                # 手动控制
                keys = pygame.key.get_pressed()
                steer = 0
                speed = 0
                if keys[pygame.K_LEFT]:
                    steer = -1
                if keys[pygame.K_RIGHT]:
                    steer = 1
                if keys[pygame.K_UP]:
                    speed = 1.0
                if keys[pygame.K_DOWN]:
                    speed = -0.5
            
            # 更新导管
            if speed != 0:
                self.catheter.move_forward(speed)
            if steer != 0:
                self.catheter.steer(steer)
            
            # 碰撞检测
            if self.check_collision():
                self.collisions += 1
                # 碰撞后稍微后退
                self.catheter.move_forward(-0.5)
            
            # 目标检测
            if self.check_goal_reached():
                print("🎉 Goal Reached!")
                self.reset()
            
            self.steps += 1
            
            # 绘制
            self.screen.fill(COLORS['background'])
            self.draw_grid()
            self.draw_target()
            
            for obs in self.obstacles:
                obs.draw(self.screen)
            
            self.draw_sensors()
            self.draw_catheter()
            self.draw_ui()
            
            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()


if __name__ == "__main__":
    sim = CatheterSimulation()
    sim.run()
