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
parser.add_argument('--dvc', type=str, default='cpu', help='running device of Sparrow: cuda / cpu')
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
parser.add_argument('--Obs_R', type=int, default=14, help='maximal obstacle radius, cm')
parser.add_argument('--Obs_V', type=int, default=30, help='maximal obstacle velocity, cm/s')
parser.add_argument('--MapObs', type=str, default=None, help="name of map file, e.g. 'map.png' or None")
parser.add_argument('--ld_a_range', type=int, default=360, help='max scanning angle of lidar (degree)')
parser.add_argument('--ld_d_range', type=int, default=300, help='max scanning distance of lidar (cm)')
parser.add_argument('--ld_num', type=int, default=72, help='number of lidar streams in each world')
parser.add_argument('--ld_GN', type=int, default=3, help='how many lidar streams are grouped for one group')
parser.add_argument('--ri', type=int, default=0, help='render index: the index of world that be rendered')
parser.add_argument('--basic_ctrl_interval', type=float, default=0.1, help='control interval (s), 0.1 means 10 Hz control frequency')
parser.add_argument('--ctrl_delay', type=int, default=0, help='control delay, in basic_ctrl_interval, 0 means no control delay')
parser.add_argument('--K', type=tuple, default=(0.55,0.6), help='K_linear, K_angular')
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
print("\nPress 'Esc' to reset environment.")

# Init Transqer agent
agent = Transqer_agent(opt)
agent.load(opt.ModelIdex)

agent.queue.clear()
s, info = envs.reset()

#加代码，一个function()
def test(relative_real_x_color, relative_real_y_color):
    return relative_real_x_color, relative_real_y_color


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://0.0.0.0:0227") # 服务器端（Server）

while True:
    # reset() env 渲染
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        agent.queue.clear()
        s, info = envs.reset()

    agent.queue.append(s)  # 将s加入时序窗口队列
    TW_s = agent.queue.get()  # 取出队列所有数据及

    if envs.win_vec:
        a = 7 * torch.ones((opt.N,), dtype=torch.long,
                           device=opt.dvc)  # 在a_space中最后一个动作选择是停止。 在离散动作空间中，a是a_space的索引，不是action本身。
    else:
        a = agent.select_action(TW_s, deterministic=False)

    print("in the step, a is: ", a)
    # 加一个发出消息的try,执行动作

    #下策? 等robo靠近下一个目标位置时再更新整个环境包括渲染??? 看看怎么把labview中的判断条件接进去?
    s, r, dw, tr, info = envs.step(a) #step里一个接收消息，直接修正s

    # 通讯
    message = socket.recv_json()
    # print("Received message:", message)  # Debugging: Print the received message
    try:
        relative_real_x_color, relative_real_y_color, t = message['relative_real_x_color'], message[
            'relative_real_y_color'], message['t']
        c1, c2 = test(relative_real_x_color, relative_real_y_color)
        t2 = t
        # c=a+b #调用函数
        data_to_send = {
            'c1': c1,
            'c2': c2,
            't2': t2
        }
        socket.send_json(data_to_send)

    except KeyError as e:
        print(f"KeyError: {e}. Message received: {message}")
        socket.send_json({'error': str(e), 'message': message})
