"""
生成3D Catheter运动演示视频
- 基座固定
- 尖端移动、弯曲
- 保存为GIF/MP4
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from PIL import Image
import io
import os

class FixedBaseCatheter:
    """固定基座的柔性导管"""
    
    def __init__(self, base_position=[0, 0, 0], num_segments=15, segment_length=8.0):
        self.base_position = np.array(base_position, dtype=float)
        self.num_segments = num_segments
        self.segment_length = segment_length
        
        # 控制点
        self.control_points = np.zeros((num_segments + 1, 3))
        self._initialize()
        
        # 弯曲参数
        self.bend_phase = 0  # 弯曲相位
        self.bend_speed = 0.15  # 弯曲速度
        
    def _initialize(self):
        """初始化为直线导管"""
        for i in range(self.num_segments + 1):
            self.control_points[i] = self.base_position + np.array([i * self.segment_length, 0, 0])
    
    def update(self, t):
        """更新导管形状 - 模拟弯曲运动"""
        self.bend_phase = t * self.bend_speed
        
        # 基座始终固定
        self.control_points[0] = self.base_position.copy()
        
        # 计算每个节段的位置
        for i in range(1, self.num_segments + 1):
            # 弯曲程度随着距离基座越远而增大
            bend_factor = (i / self.num_segments) ** 1.5
            
            # 水平方向的弯曲（左右摆动）
            yaw_angle = 0.4 * np.sin(self.bend_phase) * bend_factor
            # 垂直方向的弯曲（上下摆动）
            pitch_angle = 0.3 * np.sin(self.bend_phase * 0.7 + 1.0) * bend_factor
            
            # 计算该节段相对于前一节段的方向
            prev_point = self.control_points[i-1]
            
            # 基础方向（沿X轴）
            base_dir = np.array([1.0, 0.0, 0.0])
            
            # 应用偏航（绕Z轴旋转）
            cos_yaw, sin_yaw = np.cos(yaw_angle), np.sin(yaw_angle)
            rotated_dir = np.array([
                base_dir[0] * cos_yaw - base_dir[1] * sin_yaw,
                base_dir[0] * sin_yaw + base_dir[1] * cos_yaw,
                base_dir[2]
            ])
            
            # 应用俯仰（绕Y轴旋转）
            cos_pitch, sin_pitch = np.cos(pitch_angle), np.sin(pitch_angle)
            final_dir = np.array([
                rotated_dir[0] * cos_pitch + rotated_dir[2] * sin_pitch,
                rotated_dir[1],
                -rotated_dir[0] * sin_pitch + rotated_dir[2] * cos_pitch
            ])
            
            # 更新节点位置
            self.control_points[i] = prev_point + final_dir * self.segment_length
    
    def get_smooth_curve(self, num_points=100):
        """使用插值生成平滑曲线"""
        from scipy.interpolate import CubicSpline
        
        t = np.linspace(0, 1, len(self.control_points))
        t_new = np.linspace(0, 1, num_points)
        
        curve = np.zeros((num_points, 3))
        for dim in range(3):
            cs = CubicSpline(t, self.control_points[:, dim])
            curve[:, dim] = cs(t_new)
        
        return curve
    
    def get_tube_mesh(self, radius=2.0, resolution=12):
        """生成管状网格"""
        try:
            curve = self.get_smooth_curve(50)
        except:
            curve = self.control_points
        
        n = len(curve)
        theta = np.linspace(0, 2 * np.pi, resolution)
        
        tube_x = np.zeros((n, resolution))
        tube_y = np.zeros((n, resolution))
        tube_z = np.zeros((n, resolution))
        
        for i in range(n):
            # 计算切向量
            if i == 0:
                tangent = curve[1] - curve[0]
            elif i == n-1:
                tangent = curve[n-1] - curve[n-2]
            else:
                tangent = curve[i+1] - curve[i-1]
            
            tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
            
            # 找垂直向量
            if abs(tangent[0]) < 0.9:
                perp1 = np.cross(tangent, [1, 0, 0])
            else:
                perp1 = np.cross(tangent, [0, 1, 0])
            perp1 = perp1 / (np.linalg.norm(perp1) + 1e-6)
            perp2 = np.cross(tangent, perp1)
            
            # 生成圆周上的点
            for j, th in enumerate(theta):
                offset = radius * (np.cos(th) * perp1 + np.sin(th) * perp2)
                tube_x[i, j] = curve[i, 0] + offset[0]
                tube_y[i, j] = curve[i, 1] + offset[1]
                tube_z[i, j] = curve[i, 2] + offset[2]
        
        return tube_x, tube_y, tube_z


def create_sphere(center, radius, resolution=15):
    """创建球体表面"""
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z


def generate_video(output_path='catheter_demo.gif', total_frames=120, fps=20):
    """生成导管运动视频"""
    
    print(f"🎬 开始生成视频... 共 {total_frames} 帧")
    
    # 创建导管
    catheter = FixedBaseCatheter(
        base_position=[0, 0, 50],
        num_segments=12,
        segment_length=10
    )
    
    # 目标点
    target = np.array([140, 0, 50])
    
    # 障碍物
    obstacles = [
        {'pos': [50, 20, 55], 'r': 15, 'color': '#ef4444'},
        {'pos': [80, -15, 45], 'r': 12, 'color': '#64748b'},
        {'pos': [100, 10, 60], 'r': 10, 'color': '#ef4444'},
    ]
    
    # 存储帧图像
    frames = []
    
    # 创建图形
    fig = plt.figure(figsize=(12, 9))
    
    for frame in range(total_frames):
        fig.clear()
        ax = fig.add_subplot(111, projection='3d')
        
        # 设置背景
        fig.patch.set_facecolor('#0f0f23')
        ax.set_facecolor('#16213e')
        
        # 更新导管
        catheter.update(frame)
        
        # 获取导管数据
        try:
            curve = catheter.get_smooth_curve(80)
        except:
            curve = catheter.control_points
        
        # 绘制导管中心线（渐变色）
        colors = plt.cm.plasma(np.linspace(0.3, 0.95, len(curve)))
        for i in range(len(curve) - 1):
            ax.plot3D(
                curve[i:i+2, 0], curve[i:i+2, 1], curve[i:i+2, 2],
                color=colors[i], linewidth=5, solid_capstyle='round'
            )
        
        # 绘制管状表面
        try:
            tube_x, tube_y, tube_z = catheter.get_tube_mesh(radius=2.5, resolution=10)
            ax.plot_surface(tube_x, tube_y, tube_z, color='#ec4899', alpha=0.5, shade=True)
        except:
            pass
        
        # 绘制固定的基座
        base = catheter.base_position
        ax.scatter(*base, color='#64748b', s=200, marker='s', 
                   edgecolors='white', linewidth=2, zorder=10)
        ax.text(base[0], base[1], base[2] - 10, 'Fixed Base', 
                color='white', fontsize=9, ha='center')
        
        # 绘制移动的尖端
        tip = catheter.control_points[-1]
        ax.scatter(*tip, color='#fbbf24', s=180, marker='o', 
                   edgecolors='white', linewidth=2, zorder=10)
        
        # 绘制尖端轨迹提示
        tip_dir = catheter.control_points[-1] - catheter.control_points[-2]
        tip_dir = tip_dir / (np.linalg.norm(tip_dir) + 1e-6) * 15
        ax.quiver(tip[0], tip[1], tip[2], tip_dir[0], tip_dir[1], tip_dir[2],
                  color='#fbbf24', arrow_length_ratio=0.3, linewidth=2)
        
        # 绘制障碍物
        for obs in obstacles:
            sx, sy, sz = create_sphere(obs['pos'], obs['r'])
            ax.plot_surface(sx, sy, sz, color=obs['color'], alpha=0.6)
        
        # 绘制目标
        tx, ty, tz = create_sphere(target, 12, 20)
        ax.plot_surface(tx, ty, tz, color='#22c55e', alpha=0.4)
        ax.scatter(*target, color='#22c55e', s=80, marker='*')
        
        # 设置坐标轴
        ax.set_xlim([-20, 160])
        ax.set_ylim([-60, 60])
        ax.set_zlim([10, 90])
        ax.set_xlabel('X', color='white', fontsize=10)
        ax.set_ylabel('Y', color='white', fontsize=10)
        ax.set_zlabel('Z', color='white', fontsize=10)
        ax.tick_params(colors='white', labelsize=8)
        
        # 标题
        ax.set_title(f'3D Flexible Catheter Demo\nFrame: {frame+1}/{total_frames}', 
                     color='white', fontsize=14, fontweight='bold')
        
        # 视角（缓慢旋转）
        ax.view_init(elev=20, azim=30 + frame * 0.5)
        
        # 将图形保存到内存
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=100, facecolor='#0f0f23', 
                    bbox_inches='tight', pad_inches=0.1)
        buf.seek(0)
        frames.append(Image.open(buf).copy())
        buf.close()
        
        # 进度显示
        if (frame + 1) % 20 == 0:
            print(f"  已完成: {frame+1}/{total_frames} 帧 ({(frame+1)*100//total_frames}%)")
    
    plt.close(fig)
    
    # 保存为GIF
    print(f"💾 正在保存视频到: {output_path}")
    
    # 确定输出格式
    if output_path.endswith('.gif'):
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            duration=1000//fps,  # 毫秒
            loop=0
        )
    else:
        # 保存为多张图片（如果需要其他格式）
        for i, frame in enumerate(frames):
            frame.save(f'{output_path}_{i:04d}.png')
    
    print(f"✅ 视频已保存: {output_path}")
    print(f"   - 总帧数: {total_frames}")
    print(f"   - 帧率: {fps} fps")
    print(f"   - 时长: {total_frames/fps:.1f} 秒")
    
    return output_path


if __name__ == "__main__":
    # 生成视频
    output_file = "/workspace/catheter_obstacle_avoidance_demo/catheter_motion_demo.gif"
    generate_video(output_file, total_frames=100, fps=15)
