import zmq
from Sparrow_V2 import Sparrow_PlayGround, str2bool
from utils.Transqer import Transqer_agent
import torch, pygame
import argparse

parser = argparse.ArgumentParser()
'''Hyperparameter Setting for Transqer'''
parser.add_argument('--ModelIdex', type=int, default=2450, help='which model(e.g. 2450k.pth) to load')
parser.add_argument('--net_width', type=int, default=64, help='Linear net width')
parser.add_argument('--T', type=int, default=10, help='length of time window')
parser.add_argument('--H', type=int, default=8, help='Number of Head')
parser.add_argument('--L', type=int, default=3, help='Number of Transformer Encoder Layers')

'''Hyperparameter Setting for Sparrow'''
parser.add_argument('--dvc', type=str, default='cuda', help='running device of Sparrow: cuda / cpu')
parser.add_argument('--action_type', type=str, default='Discrete', help='Action type: Discrete / Continuous')
parser.add_argument('--window_size', type=int, default=800, help='size of the map')
parser.add_argument('--D', type=int, default=400, help='maximal local planning distance:366*1.414')
parser.add_argument('--N', type=int, default=1, help='number of vectorized environments')
parser.add_argument('--O', type=int, default=15, help='number of obstacles in each environment')
parser.add_argument('--RdON', type=str2bool, default=False, help='whether to randomize the Number of dynamic obstacles')
parser.add_argument('--ScOV', type=str2bool, default=False, help='whether to scale the maximal velocity of dynamic obstacles')
parser.add_argument('--RdOV', type=str2bool, default=True, help='whether to randomize the Velocity of dynamic obstacles')
parser.add_argument('--RdOT', type=str2bool, default=True, help='whether to randomize the Type of dynamic obstacles')
parser.add_argument('--RdOR', type=str2bool, default=True, help='whether to randomize the Radius of obstacles')
parser.add_argument('--Obs_R', type=int, default=12, help='maximal obstacle radius, cm')
parser.add_argument('--Obs_V', type=int, default=11, help='maximal obstacle velocity, cm/s')
parser.add_argument('--MapObs', type=str, default=None, help="name of map file, e.g. 'map.png' or None")
parser.add_argument('--ld_a_range', type=int, default=360, help='max scanning angle of lidar (degree)')
parser.add_argument('--ld_d_range', type=int, default=300, help='max scanning distance of lidar (cm)')
parser.add_argument('--ld_num', type=int, default=72, help='number of lidar streams in each world')
parser.add_argument('--ld_GN', type=int, default=3, help='how many lidar streams are grouped for one group')
parser.add_argument('--ri', type=int, default=0, help='render index: the index of world that be rendered')
parser.add_argument('--basic_ctrl_interval', type=float, default=0.15, help='control interval (s), 0.1 means 10 Hz control frequency')
parser.add_argument('--ctrl_delay', type=int, default=0, help='control delay, in basic_ctrl_interval, 0 means no control delay')
parser.add_argument('--K', type=tuple, default=(0.05,0.05), help='K_linear, K_angular')
parser.add_argument('--draw_auxiliary', type=str2bool, default=False, help='whether to draw auxiliary infos')
parser.add_argument('--render_speed', type=str, default='fast', help='fast / slow / real')
parser.add_argument('--max_ep_steps', type=int, default=500, help='maximum episodic steps')
parser.add_argument('--noise', type=str2bool, default=True, help='whether to add noise to the observations')
parser.add_argument('--DR', type=str2bool, default=True, help='whether to use Domain Randomization')
parser.add_argument('--DR_freq', type=int, default=int(3.2e3), help='frequency of Domain Randomization, in total steps')
parser.add_argument('--compile', type=str2bool, default=False, help='whether to use torch.compile to boost simulation speed')
opt = parser.parse_args()

opt.render_mode = 'human'

opt.dvc = torch.device(opt.dvc)

envs = Sparrow_PlayGround(**vars(opt))
opt.state_dim = envs.state_dim
opt.action_dim = envs.action_dim
#print("\nPress 'Esc' to reset environment.")


# Init Transqer agent
agent = Transqer_agent(opt)
agent.load(opt.ModelIdex)

agent.queue.clear()
s, info = envs.reset()
r = 0
dw = False
tr = False
a = 7 * torch.ones((opt.N,), dtype=torch.long, device=opt.dvc)
arange_constant = envs.arange_constant
K = envs.K
ctrl_interval = envs.ctrl_interval


'''def action_to_expected_car_position(action_index, temp_car_state, arange_constant, ctrl_interval, K, a_space):
    a_space = torch.tensor([[0.2*envs.v_linear_max , envs.v_angular_max],  # 动作 0：低速前进 + 右转
                                     [envs.v_linear_max , envs.v_angular_max], # 动作 1：全速前进 + 右转
                                     [envs.v_linear_max, 0], # 动作 2：全速前进
                                     [envs.v_linear_max, - envs.v_angular_max],     # 动作 3：全速前进 + 左转
                                     [0.2*envs.v_linear_max, - envs.v_angular_max], # 动作 4：低速前进 + 左转
                                     [-envs.v_linear_max, 0],  # 动作 5：全速后退
                                     [0.1*envs.v_linear_max, 0], # 动作 6：减速
                                     [0., 0.]], device = envs.dvc) # 动作 7：停止
    
    #[dx,dy,orientation,v_l,v_a]
    #self.car_state[:,3:5] = self.K * self.car_state[:,3:5] + (1-self.K)*self.a_space[self.arange_constant,a] # self.a_space[a] is (N,2)
        #return torch.stack((self.car_state[:,3],self.car_state[:,3],self.car_state[:,4]),dim=1) # [v_l, v_l, v_a], (N,3)
    temp = K * temp_car_state[:,3:5] + (1-K)*a_space[arange_constant,action_index] 
    velocity = torch.stack((temp[:,0], temp[:,0]), dim=1) # [v_l, v_l], (N,2)
    next_car_position = temp_car_state[:,0:2] + ctrl_interval * velocity * torch.stack((torch.cos(temp_car_state[:,2]),-torch.sin(temp_car_state[:,2])), dim=1)
    
    return next_car_position'''

#ipconfig 查看IPv4地址
#netsh advfirewall set allprofiles state off 临时关闭防火墙
context = zmq.Context()
socket_client1 = context.socket(zmq.REP)
socket_client1.bind("tcp://0.0.0.0:0226")
#netstat -ano | findstr 0225  用于查找是否有这个监听窗口

a_space = torch.tensor([[0.2*envs.v_linear_max , envs.v_angular_max],  # 动作 0：低速前进 + 右转
                                     [envs.v_linear_max , envs.v_angular_max], # 动作 1：全速前进 + 右转
                                     [envs.v_linear_max, 0], # 动作 2：全速前进
                                     [envs.v_linear_max, - envs.v_angular_max],     # 动作 3：全速前进 + 左转
                                     [0.2*envs.v_linear_max, - envs.v_angular_max], # 动作 4：低速前进 + 左转
                                     [-envs.v_linear_max, 0],  # 动作 5：全速后退
                                     [0.1*envs.v_linear_max, 0], # 动作 6：减速
                                     [0., 0.]], device = envs.dvc) # 动作 7：停止

a_space_numpy = a_space.cpu().numpy()
#print("a_space_numpy: ", a_space_numpy)

import numpy as np
def action_to_expected_car_position(a, temp_car_state, arange_constant, ctrl_interval, K, a_space):
    temp = a_space[a]
    
    # 速度向量 [v_l, v_l] 
    velocity = np.array(temp[0], temp[0])
    
    # 方向向量 [cos(theta), -sin(theta)] (2)
    direction = np.array(np.cos(temp_car_state[2]), -np.sin(temp_car_state[2]))
    
    # 计算新位置: position + ctrl_interval * velocity * direction
    next_car_position = temp_car_state[0:2] + ctrl_interval * velocity * direction
    
    return next_car_position


print("finish init . ")

next_relative_desired_point = [20, 20]

while True:
    # reset() env 渲染
    '''keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        agent.queue.clear()
        s, info = envs.reset()'''
    
    # 通讯
    message1 = socket_client1.recv_json()
    #print("Received message1:", message1)  # Debugging: Print the received message
    try:
        print("----------------------------------------------------------------------------------------------------------")
        print("step_counter_vec: ", envs.step_counter_vec, "; target_point: ", envs.target_point, "; d2target_now: ", envs.d2target_now)
        relative_real_x_color, relative_real_y_color = message1['relative_real_x_color'], message1['relative_real_y_color']

        s, r, dw, tr, info = envs.step(a, relative_real_x_color, relative_real_y_color) 

        agent.queue.append(s)  # 将s加入时序窗口队列
        TW_s = agent.queue.get()  # 取出队列所有数据及

        if envs.win_vec:
            a = 7 * torch.ones((opt.N,), dtype=torch.long,
                            device=opt.dvc)  # 在a_space中最后一个动作选择是停止。 在离散动作空间中，a是a_space的索引，不是action本身。
        else:
            a = agent.select_action(TW_s, deterministic=False)
       
        temp_car_state =  info['abs_car_state']
        #print("temp_car_state: ", temp_car_state)#value1 = self.car_state[0, 0].item()  # 转为Python float   #value2 = self.car_state[0, 1].item()  # 转为Python float
        #print("a: ", a)
        a_numpy =  a.cpu().numpy()[0]
        #print("a_numpy: ", a_numpy)
        temp_car_state_numpy = temp_car_state.cpu().numpy()[0]
        print("temp_car_state_numpy: ", temp_car_state_numpy)
        arange_constant_numpy = arange_constant.cpu().numpy()[0]
        #print("arange_constant_numpy: ", arange_constant_numpy)
        ctrl_interval_numpy = ctrl_interval.cpu().numpy()[0]
        #print("ctrl_interval_numpy: ", ctrl_interval_numpy)
        K_numpy = K.cpu().numpy()[0]
        #print("K_numpy: ", K_numpy)
        translation = temp_car_state_numpy[0:2] - [relative_real_x_color, relative_real_y_color]
        #print("translation: ", translation)
        next_car_position = action_to_expected_car_position(a_numpy, temp_car_state_numpy, arange_constant_numpy, ctrl_interval_numpy, K_numpy, a_space_numpy)
        #next_car_position = action_to_expected_car_position(a, temp_car_state, arange_constant, ctrl_interval, K, a_space)
        if envs.step_counter_vec < 151:
            next_relative_desired_point = next_car_position - translation
        else:
            envs.car_refresh_counter += 1
            if envs.car_refresh_counter > envs.car_refresh_interval:
                next_relative_desired_point = next_car_position - translation
                envs.car_refresh_counter = 0

        print("next_car_position: ", next_car_position)
        print("next_relative_desired_point: ", next_relative_desired_point)
        #action_index_value = a.item()
        #action_Discrete = a_space_numpy[action_index_value]  #print("action_Discrete: ", action_Discrete)
        data_to_send1 = {
            'c1': float(next_relative_desired_point[0]),
            'c2': float(next_relative_desired_point[1]),
        }
        socket_client1.send_json(data_to_send1)

    except KeyError as e:
        print(f"KeyError: {e}. Message received: {message1}")
        socket_client1.send_json({'error': str(e), 'message': message1})
    

    
