import zmq
import time
PORT = 9560
context = zmq.Context()
socket = context.socket(zmq.REP)

socket.bind(f"tcp://*:{PORT}")

print(f"на порте {PORT}")

while True:
    message = socket.recv()
    print(f"[SERVER] Received from Android: {message.decode('utf-8')}")

    time.sleep(1)

    reply = "Hello from Python server!"
    socket.send(reply.encode("utf-8"))

