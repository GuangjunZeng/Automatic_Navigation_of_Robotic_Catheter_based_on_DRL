"""
3D Catheter 静态展示 - 展示Matplotlib 3D能力
无需额外依赖，只需要numpy和matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def create_catheter_curve(num_points=100):
    """
    创建一个弯曲的3D导管曲线
    使用参数方程生成螺旋式弯曲路径
    """
    t = np.linspace(0, 4 * np.pi, num_points)
    
    # 创建一个S形弯曲的导管路径
    x = t * 5  # 沿X轴延伸
    y = 20 * np.sin(t * 0.5) * np.exp(-t * 0.05)  # Y方向的弯曲
    z = 15 * np.cos(t * 0.3) * np.exp(-t * 0.03) + 50  # Z方向的弯曲，基准高度50
    
    return x, y, z

def create_sphere(center, radius, resolution=20):
    """创建球体表面网格"""
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z

def create_tube_around_curve(x, y, z, radius=2, resolution=12):
    """
    在曲线周围创建管状表面（模拟导管的3D外形）
    """
    n = len(x)
    theta = np.linspace(0, 2 * np.pi, resolution)
    
    # 存储管状表面的点
    tube_x = np.zeros((n, resolution))
    tube_y = np.zeros((n, resolution))
    tube_z = np.zeros((n, resolution))
    
    for i in range(n):
        # 计算当前点的切向量
        if i == 0:
            tangent = np.array([x[1]-x[0], y[1]-y[0], z[1]-z[0]])
        elif i == n-1:
            tangent = np.array([x[n-1]-x[n-2], y[n-1]-y[n-2], z[n-1]-z[n-2]])
        else:
            tangent = np.array([x[i+1]-x[i-1], y[i+1]-y[i-1], z[i+1]-z[i-1]])
        
        tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
        
        # 找一个垂直于切向量的向量
        if abs(tangent[0]) < 0.9:
            perp1 = np.cross(tangent, [1, 0, 0])
        else:
            perp1 = np.cross(tangent, [0, 1, 0])
        perp1 = perp1 / (np.linalg.norm(perp1) + 1e-6)
        
        perp2 = np.cross(tangent, perp1)
        perp2 = perp2 / (np.linalg.norm(perp2) + 1e-6)
        
        # 在垂直平面上画圆
        for j, th in enumerate(theta):
            offset = radius * (np.cos(th) * perp1 + np.sin(th) * perp2)
            tube_x[i, j] = x[i] + offset[0]
            tube_y[i, j] = y[i] + offset[1]
            tube_z[i, j] = z[i] + offset[2]
    
    return tube_x, tube_y, tube_z

def main():
    """主函数 - 创建3D导管可视化"""
    
    # 创建图形
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 设置深色背景
    fig.patch.set_facecolor('#0f0f23')
    ax.set_facecolor('#1a1a2e')
    
    # ===== 1. 创建导管 =====
    catheter_x, catheter_y, catheter_z = create_catheter_curve(150)
    
    # 方法1: 使用渐变色线条表示导管
    points = np.array([catheter_x, catheter_y, catheter_z]).T.reshape(-1, 1, 3)
    
    # 使用渐变颜色绘制导管
    colors = plt.cm.plasma(np.linspace(0.2, 0.9, len(catheter_x)))
    for i in range(len(catheter_x) - 1):
        ax.plot3D(
            catheter_x[i:i+2], 
            catheter_y[i:i+2], 
            catheter_z[i:i+2],
            color=colors[i], 
            linewidth=4,
            solid_capstyle='round'
        )
    
    # 方法2: 创建管状3D表面（更真实的导管外形）
    tube_x, tube_y, tube_z = create_tube_around_curve(
        catheter_x[::3], catheter_y[::3], catheter_z[::3], 
        radius=1.5, resolution=16
    )
    ax.plot_surface(
        tube_x, tube_y, tube_z,
        color='#ec4899',  # 粉色
        alpha=0.6,
        shade=True
    )
    
    # ===== 2. 绘制导管尖端 =====
    tip = (catheter_x[-1], catheter_y[-1], catheter_z[-1])
    ax.scatter(*tip, color='#fbbf24', s=200, marker='o', 
               edgecolors='white', linewidth=2, label='Catheter Tip', zorder=5)
    
    # 绘制尖端方向指示
    tip_dir = np.array([
        catheter_x[-1] - catheter_x[-5],
        catheter_y[-1] - catheter_y[-5],
        catheter_z[-1] - catheter_z[-5]
    ])
    tip_dir = tip_dir / np.linalg.norm(tip_dir) * 10
    ax.quiver(tip[0], tip[1], tip[2], tip_dir[0], tip_dir[1], tip_dir[2],
              color='#fbbf24', arrow_length_ratio=0.3, linewidth=2)
    
    # ===== 3. 绘制导管基座 =====
    base = (catheter_x[0], catheter_y[0], catheter_z[0])
    ax.scatter(*base, color='#64748b', s=150, marker='s', 
               edgecolors='white', linewidth=2, label='Base')
    
    # ===== 4. 绘制障碍物 =====
    obstacles = [
        {'pos': [30, 15, 55], 'r': 12, 'color': '#ef4444', 'label': 'Dynamic Obs'},
        {'pos': [45, -10, 48], 'r': 10, 'color': '#64748b', 'label': 'Static Obs'},
        {'pos': [55, 8, 60], 'r': 8, 'color': '#ef4444', 'label': None},
        {'pos': [25, -5, 42], 'r': 9, 'color': '#64748b', 'label': None},
    ]
    
    for i, obs in enumerate(obstacles):
        sx, sy, sz = create_sphere(obs['pos'], obs['r'], 15)
        ax.plot_surface(sx, sy, sz, color=obs['color'], alpha=0.7)
        if obs['label']:
            ax.scatter([], [], [], color=obs['color'], s=100, label=obs['label'])
    
    # ===== 5. 绘制目标点 =====
    target = [65, 0, 50]
    target_radius = 8
    tx, ty, tz = create_sphere(target, target_radius, 20)
    ax.plot_surface(tx, ty, tz, color='#22c55e', alpha=0.5)
    ax.scatter(*target, color='#22c55e', s=100, marker='*', label='Target')
    
    # ===== 6. 绘制传感器射线 =====
    sensor_directions = [
        ([1, 0, 0], '#38bdf8', 'Front'),
        ([0, 1, 0], '#38bdf8', None),
        ([0, -1, 0], '#38bdf8', None),
        ([0, 0, 1], '#38bdf8', None),
        ([0, 0, -1], '#38bdf8', None),
    ]
    
    sensor_length = 15
    for direction, color, label in sensor_directions:
        end = [tip[i] + direction[i] * sensor_length for i in range(3)]
        ax.plot3D([tip[0], end[0]], [tip[1], end[1]], [tip[2], end[2]],
                 color=color, linewidth=1.5, linestyle='--', alpha=0.6)
    ax.plot3D([], [], [], color='#38bdf8', linewidth=1.5, linestyle='--', label='Sensors')
    
    # ===== 7. 绘制边界框 =====
    bounds = [[-5, 75], [-30, 30], [30, 75]]
    # 绘制边界线框
    for x in bounds[0]:
        for y in bounds[1]:
            ax.plot3D([x, x], [y, y], bounds[2], color='white', alpha=0.2, linewidth=0.5)
    for x in bounds[0]:
        for z in bounds[2]:
            ax.plot3D([x, x], bounds[1], [z, z], color='white', alpha=0.2, linewidth=0.5)
    for y in bounds[1]:
        for z in bounds[2]:
            ax.plot3D(bounds[0], [y, y], [z, z], color='white', alpha=0.2, linewidth=0.5)
    
    # ===== 设置样式 =====
    ax.set_xlabel('X (mm)', color='white', fontsize=12)
    ax.set_ylabel('Y (mm)', color='white', fontsize=12)
    ax.set_zlabel('Z (mm)', color='white', fontsize=12)
    
    ax.set_xlim(bounds[0])
    ax.set_ylim(bounds[1])
    ax.set_zlim(bounds[2])
    
    # 设置刻度颜色
    ax.tick_params(colors='white')
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('white')
    ax.yaxis.pane.set_edgecolor('white')
    ax.zaxis.pane.set_edgecolor('white')
    
    # 标题
    ax.set_title('3D Flexible Catheter Obstacle Avoidance Demo\n', 
                color='white', fontsize=16, fontweight='bold')
    
    # 图例
    ax.legend(loc='upper left', facecolor='#1a1a2e', edgecolor='white', 
              labelcolor='white', fontsize=10)
    
    # 设置视角
    ax.view_init(elev=25, azim=45)
    
    # 添加说明文字
    info_text = """
    Catheter Simulation Features:
    • Flexible multi-segment catheter
    • 3D obstacle avoidance
    • Sensor-based navigation
    • Dynamic & static obstacles
    """
    fig.text(0.02, 0.02, info_text, color='white', fontsize=9,
             family='monospace', verticalalignment='bottom')
    
    plt.tight_layout()
    
    # 保存图片
    plt.savefig('/workspace/catheter_obstacle_avoidance_demo/catheter_3d_demo.png', 
                dpi=150, facecolor='#0f0f23', edgecolor='none', bbox_inches='tight')
    print("✅ 图片已保存到: /workspace/catheter_obstacle_avoidance_demo/catheter_3d_demo.png")
    
    # 显示
    plt.show()


if __name__ == "__main__":
    main()
