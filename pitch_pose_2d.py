import numpy as np
import quaternion
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from pose_listen import start_ble_listener, pose_data
from time import sleep

ARM_LENGTH = 0.3 # m


start_ble_listener()

def estimate_wrist_position(pitch_angle, forearm_length):

    angle_rad = pitch_angle * math.pi / 180
    x = forearm_length * math.cos(angle_rad)
    y = forearm_length * math.sin(angle_rad)

    return x,y



# Follows flowchart from associated paper 'Human Arm Motion Capture Using IMU Sensors'
print('hi')

fig = plt.figure()

ax = fig.add_subplot(111)
origin = np.array([0,0])
x,y = estimate_wrist_position(30,ARM_LENGTH)
print(x,y)
vector = np.array([x,y])
print(abs(vector))


quiver = ax.quiver(*origin,*vector,angles='xy', scale_units='xy', scale=1)
ax.set_xlim([0,1])
ax.set_ylim([-1,1])
ax.set_aspect('equal',adjustable='box')

def update(frame):
    pitch = pose_data["pitch"]
    x,y = estimate_wrist_position(pitch,ARM_LENGTH)
    quiver.set_UVC(x,y)
    return quiver,

ani = FuncAnimation(fig,update,frames=100,interval=50,blit=True,repeat=True)

plt.show()

