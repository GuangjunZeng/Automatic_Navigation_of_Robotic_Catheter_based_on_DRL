import zmq

#加代码，一个function()


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:0216")

while True:
    message = socket.recv_json()
    # print("Received message:", message)  # Debugging: Print the received message
    try:
        a, b = message['a'], message['b']

        c=a+b #调用函数


        socket.send_json({'c': c})

    except KeyError as e:
        print(f"KeyError: {e}. Message received: {message}")
        socket.send_json({'error': str(e), 'message': message})