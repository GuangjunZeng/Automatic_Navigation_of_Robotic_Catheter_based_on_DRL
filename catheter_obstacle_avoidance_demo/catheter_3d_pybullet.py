"""
3D Catheter Obstacle Avoidance Simulation - PyBullet Version
基于PyBullet的3D柔性导管物理仿真

安装依赖: pip install pybullet numpy

PyBullet特点:
- 真实物理引擎（碰撞检测、重力、摩擦）
- 支持软体（Soft Body）模拟柔性导管
- 适合强化学习训练
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math


class CatheterPyBullet:
    """基于PyBullet的柔性导管类"""
    
    def __init__(self, physics_client, base_position=[0, 0, 0.5], 
                 num_segments=10, segment_length=0.08):
        self.client = physics_client
        self.num_segments = num_segments
        self.segment_length = segment_length
        self.segment_radius = 0.015  # 导管半径
        
        # 创建导管（由多个刚体链接组成）
        self.links = []
        self.joints = []
        self.catheter_id = self._create_catheter(base_position)
        
        # 控制参数
        self.max_force = 50
        self.max_velocity = 2.0
    
    def _create_catheter(self, base_pos):
        """创建多段刚体链接的导管"""
        
        # 碰撞形状 - 胶囊体
        collision_shape = p.createCollisionShape(
            p.GEOM_CAPSULE,
            radius=self.segment_radius,
            height=self.segment_length
        )
        
        # 视觉形状 - 胶囊体（粉色）
        visual_shape = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=self.segment_radius,
            length=self.segment_length,
            rgbaColor=[0.9, 0.3, 0.6, 1]  # 粉紫色
        )
        
        # 基座（固定）
        base_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        base_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, 
                                          rgbaColor=[0.3, 0.3, 0.3, 1])
        
        # 创建多体系统
        link_masses = [0.01] * self.num_segments  # 每段质量
        link_collision_shapes = [collision_shape] * self.num_segments
        link_visual_shapes = [visual_shape] * self.num_segments
        link_positions = []
        link_orientations = []
        link_inertial_frame_positions = []
        link_inertial_frame_orientations = []
        link_parent_indices = []
        link_joint_types = []
        link_joint_axes = []
        
        for i in range(self.num_segments):
            # 每段相对于父段的位置
            link_positions.append([self.segment_length, 0, 0])
            link_orientations.append([0, 0, 0, 1])
            link_inertial_frame_positions.append([0, 0, 0])
            link_inertial_frame_orientations.append([0, 0, 0, 1])
            link_parent_indices.append(i)  # 连接到前一段
            link_joint_types.append(p.JOINT_SPHERICAL)  # 球关节，允许3自由度旋转
            link_joint_axes.append([0, 0, 1])
        
        # 创建多体
        catheter_id = p.createMultiBody(
            baseMass=0,  # 基座固定
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=base_pos,
            baseOrientation=[0, 0, 0, 1],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collision_shapes,
            linkVisualShapeIndices=link_visual_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_frame_positions,
            linkInertialFrameOrientations=link_inertial_frame_orientations,
            linkParentIndices=link_parent_indices,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes
        )
        
        # 设置关节阻尼（模拟柔性）
        for i in range(self.num_segments):
            p.changeDynamics(
                catheter_id, i,
                jointDamping=0.5,
                linearDamping=0.5,
                angularDamping=0.5
            )
        
        return catheter_id
    
    def get_tip_position(self):
        """获取导管尖端位置"""
        link_state = p.getLinkState(self.catheter_id, self.num_segments - 1)
        return np.array(link_state[0])
    
    def get_tip_orientation(self):
        """获取导管尖端方向"""
        link_state = p.getLinkState(self.catheter_id, self.num_segments - 1)
        return np.array(link_state[1])
    
    def get_all_positions(self):
        """获取所有段的位置"""
        positions = []
        base_pos, _ = p.getBasePositionAndOrientation(self.catheter_id)
        positions.append(np.array(base_pos))
        
        for i in range(self.num_segments):
            link_state = p.getLinkState(self.catheter_id, i)
            positions.append(np.array(link_state[0]))
        
        return positions
    
    def apply_tip_force(self, force):
        """在尖端施加力"""
        p.applyExternalForce(
            self.catheter_id,
            self.num_segments - 1,
            force,
            self.get_tip_position(),
            p.WORLD_FRAME
        )
    
    def control_tip(self, target_velocity):
        """控制尖端速度"""
        # 获取当前尖端状态
        tip_pos = self.get_tip_position()
        
        # 计算需要的力
        force = np.array(target_velocity) * self.max_force
        self.apply_tip_force(force)
    
    def bend_tip(self, pitch=0, yaw=0):
        """弯曲导管尖端
        pitch: 俯仰 (上下)
        yaw: 偏航 (左右)
        """
        # 在最后几个关节施加扭矩
        for i in range(max(0, self.num_segments - 3), self.num_segments):
            torque = [pitch * 0.1, yaw * 0.1, 0]
            p.applyExternalTorque(
                self.catheter_id, i,
                torque,
                p.LINK_FRAME
            )


class Obstacle3DPyBullet:
    """PyBullet障碍物类"""
    
    def __init__(self, physics_client, position, radius=0.1, is_dynamic=False):
        self.client = physics_client
        self.position = np.array(position)
        self.radius = radius
        self.is_dynamic = is_dynamic
        
        # 创建障碍物
        self.body_id = self._create_obstacle()
        
        if is_dynamic:
            # 设置随机初始速度
            velocity = np.random.uniform(-0.3, 0.3, 3)
            velocity[2] *= 0.2  # Z方向速度较小
            p.resetBaseVelocity(self.body_id, velocity.tolist())
    
    def _create_obstacle(self):
        """创建球形障碍物"""
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        
        color = [0.9, 0.2, 0.2, 1] if self.is_dynamic else [0.5, 0.5, 0.5, 1]
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, 
                                           rgbaColor=color)
        
        mass = 0.1 if self.is_dynamic else 0  # 静态障碍物质量为0
        
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.position.tolist()
        )
        
        if self.is_dynamic:
            p.changeDynamics(body_id, -1, 
                           linearDamping=0.1,
                           restitution=0.9)  # 高弹性
        
        return body_id
    
    def get_position(self):
        """获取当前位置"""
        pos, _ = p.getBasePositionAndOrientation(self.body_id)
        return np.array(pos)
    
    def constrain_to_bounds(self, bounds):
        """将障碍物约束在边界内"""
        if self.is_dynamic:
            pos = self.get_position()
            vel, _ = p.getBaseVelocity(self.body_id)
            vel = list(vel)
            
            for i in range(3):
                if pos[i] < bounds[i][0] or pos[i] > bounds[i][1]:
                    vel[i] *= -1
            
            p.resetBaseVelocity(self.body_id, vel)


class CatheterSimulationPyBullet:
    """PyBullet导管仿真主类"""
    
    def __init__(self, gui=True):
        # 初始化PyBullet
        if gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        else:
            self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # 物理参数
        p.setGravity(0, 0, -0.5)  # 较小的重力
        p.setTimeStep(1/240)
        
        # 创建地面
        self.ground_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(self.ground_id, -1, rgbaColor=[0.1, 0.1, 0.15, 1])
        
        # 创建导管
        self.catheter = CatheterPyBullet(
            self.client, 
            base_position=[0, 0, 0.5],
            num_segments=12,
            segment_length=0.06
        )
        
        # 创建目标
        self.target_pos = np.array([1.5, 0, 0.5])
        self.target_radius = 0.15
        self._create_target()
        
        # 创建障碍物
        self.obstacles = self._create_obstacles()
        
        # 边界
        self.bounds = [(-0.5, 2.0), (-0.8, 0.8), (0.1, 1.0)]
        
        # 状态
        self.auto_mode = True
        self.steps = 0
        
        # 设置相机
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.7, 0, 0.5]
        )
        
        # 添加调试参数滑块
        self.speed_slider = p.addUserDebugParameter("Speed", 0, 2, 1)
        self.auto_slider = p.addUserDebugParameter("Auto Mode", 0, 1, 1)
    
    def _create_target(self):
        """创建目标区域"""
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE, 
            radius=self.target_radius,
            rgbaColor=[0.2, 0.8, 0.3, 0.5]
        )
        self.target_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.target_pos.tolist()
        )
    
    def _create_obstacles(self):
        """创建障碍物"""
        obstacles = [
            # 静态障碍物
            Obstacle3DPyBullet(self.client, [0.4, 0.2, 0.5], radius=0.12),
            Obstacle3DPyBullet(self.client, [0.7, -0.15, 0.6], radius=0.1),
            Obstacle3DPyBullet(self.client, [1.0, 0.25, 0.45], radius=0.13),
            Obstacle3DPyBullet(self.client, [1.2, -0.2, 0.55], radius=0.09),
            
            # 动态障碍物
            Obstacle3DPyBullet(self.client, [0.5, 0, 0.7], radius=0.08, is_dynamic=True),
            Obstacle3DPyBullet(self.client, [0.9, 0.1, 0.4], radius=0.07, is_dynamic=True),
        ]
        return obstacles
    
    def sense_environment(self):
        """感知环境"""
        tip_pos = self.catheter.get_tip_position()
        
        # 检测各方向障碍物距离
        directions = {
            'front': np.array([1, 0, 0]),
            'left': np.array([0, 1, 0]),
            'right': np.array([0, -1, 0]),
            'up': np.array([0, 0, 1]),
            'down': np.array([0, 0, -1]),
        }
        
        distances = {}
        for name, direction in directions.items():
            # 使用射线检测
            ray_start = tip_pos
            ray_end = tip_pos + direction * 0.5
            
            result = p.rayTest(ray_start.tolist(), ray_end.tolist())
            if result[0][0] != -1:  # 击中物体
                distances[name] = result[0][2] * 0.5  # 距离比例 * 射线长度
            else:
                distances[name] = 0.5
        
        return distances
    
    def auto_control(self):
        """自动避障控制"""
        tip_pos = self.catheter.get_tip_position()
        
        # 到目标的方向
        to_target = self.target_pos - tip_pos
        dist_to_target = np.linalg.norm(to_target)
        to_target_norm = to_target / (dist_to_target + 1e-6)
        
        # 感知障碍物
        distances = self.sense_environment()
        
        # 基础控制：朝向目标
        control = to_target_norm * 0.5
        
        # 避障修正
        if distances['front'] < 0.2:
            # 前方有障碍，减速并转向
            control *= 0.2
            if distances['left'] > distances['right']:
                control[1] += 0.3
            else:
                control[1] -= 0.3
            if distances['up'] > distances['down']:
                control[2] += 0.2
            else:
                control[2] -= 0.2
        
        return control
    
    def check_goal(self):
        """检查是否到达目标"""
        tip_pos = self.catheter.get_tip_position()
        dist = np.linalg.norm(tip_pos - self.target_pos)
        return dist < self.target_radius
    
    def reset(self):
        """重置仿真"""
        p.resetSimulation()
        p.setGravity(0, 0, -0.5)
        
        self.ground_id = p.loadURDF("plane.urdf")
        p.changeVisualShape(self.ground_id, -1, rgbaColor=[0.1, 0.1, 0.15, 1])
        
        self.catheter = CatheterPyBullet(
            self.client,
            base_position=[0, 0, 0.5],
            num_segments=12,
            segment_length=0.06
        )
        
        self._create_target()
        self.obstacles = self._create_obstacles()
        self.steps = 0
    
    def run(self):
        """运行仿真"""
        print("=" * 50)
        print("PyBullet 3D Catheter Simulation")
        print("=" * 50)
        print("\nControls (in GUI):")
        print("  - Use sliders to adjust parameters")
        print("  - Mouse: Rotate/Zoom view")
        print("  - R key: Reset simulation")
        print("\n" + "=" * 50)
        
        try:
            while True:
                # 读取参数
                speed = p.readUserDebugParameter(self.speed_slider)
                self.auto_mode = p.readUserDebugParameter(self.auto_slider) > 0.5
                
                # 约束动态障碍物
                for obs in self.obstacles:
                    obs.constrain_to_bounds(self.bounds)
                
                # 控制
                if self.auto_mode:
                    control = self.auto_control()
                    self.catheter.control_tip(control * speed)
                
                # 目标检测
                if self.check_goal():
                    print(f"🎉 Goal reached in {self.steps} steps!")
                    time.sleep(1)
                    self.reset()
                
                # 键盘检测
                keys = p.getKeyboardEvents()
                if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                    self.reset()
                if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
                    break
                
                # 步进仿真
                p.stepSimulation()
                self.steps += 1
                
                time.sleep(1/60)
                
        except KeyboardInterrupt:
            pass
        finally:
            p.disconnect()


if __name__ == "__main__":
    sim = CatheterSimulationPyBullet(gui=True)
    sim.run()
