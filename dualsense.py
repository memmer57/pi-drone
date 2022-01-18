from pydualsense import *
import socket
import time

UDP_IP = "10.42.0.209"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def emergency_stop():
    for i in range(1000):
        sock.sendto(b"stop", (UDP_IP, UDP_PORT))
        print("stop")

def convert(min, max, value):
    if min<0:
        value -= min
        max -= min
        min = 0
    value = max-value
    percentage = value/max
    return 6 + 3*percentage

# create dualsense
dualsense = pydualsense()
# find device and initialize
dualsense.init()

# read controller state until R1 is pressed
while True:
    LX = (-convert(-127, 128, dualsense.state.LX) + 7.5) * 20
    LY = convert(-127, 128, dualsense.state.LY) - 1.5
    RX = (-convert(-127, 128, dualsense.state.RX) + 7.5) * 10
    RY = (convert(-127, 128, dualsense.state.RY) - 7.5) * 10
    R1 = dualsense.state.R1
    L1 = dualsense.state.L1
    stop = dualsense.state.touchBtn
    
    if L1:
        emergency_stop()
        break
    print(LY, RX, RY, LX)
    sock.sendto(str(LY).encode('utf-8') + b" " + str(RX).encode('utf-8') + b" " + str(RY).encode('utf-8') + b" " + str(LX).encode('utf-8'), (UDP_IP, UDP_PORT))
    time.sleep(0.01)

# close device
dualsense.close()
