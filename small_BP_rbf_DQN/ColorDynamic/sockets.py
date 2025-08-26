
# sockets.py
import zmq
from functools import partial

# 定义全局套接字变量（初始化为None）
global_socket_client1 = None


def init_sockets():
    """初始化全局套接字（由主程序调用）"""
    global global_socket_client1
    if global_socket_client1 is None:
        context = zmq.Context()
        global_socket_client1 = context.socket(zmq.REP)
        global_socket_client1.bind("tcp://0.0.0.0:0226")
        print("first chance, init_socket of 0226")


def get_client1():
    init_sockets()
    return global_socket_client1
  