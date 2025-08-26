
# sockets.py
import zmq

# 定义全局套接字变量（初始化为None）
global_socket_client1 = None


def init_sockets():
    """初始化全局套接字（由主程序调用）"""
    global global_socket_client1
    context = zmq.Context()
    global_socket_client1 = context.socket(zmq.REP)
    global_socket_client1.bind("tcp://0.0.0.0:0226")
    
  