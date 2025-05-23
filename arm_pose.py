import numpy as np
import quaternion
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pose_listen import start_ble_listener, pose_data
from time import sleep

ARM_LENGTH = 0.3 # m

#start_ble_listener()

def estimate_wrist_position(pitch_angle, forearm_length):

    angle_rad = pitch_angle * 2 * math.pi / 180
    x = forearm_length * math.cos(angle_rad)
    y = 0
    z = forearm_length * math.sin(angle_rad)

    return x,y,z

# Follows flowchart from associated paper 'Human Arm Motion Capture Using IMU Sensors'

lims = [-1,1]

fig = plt.figure()

ax = fig.add_subplot(111,projection='3d')
origin = np.array([0,0,0])
x,y,z = estimate_wrist_position(30,ARM_LENGTH)
print(x,y,z)
vector = np.array([x,y,z])
print(abs(vector))

ax.quiver(*origin,*vector,length=np.linalg.norm(vector))
ax.set_xlim(lims)
ax.set_ylim(lims)
ax.set_zlim(lims)

plt.show()

