import numpy as np
import quaternion
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from pose_listen import start_ble_listener, pose_data
from time import sleep

ARM_LENGTH = 0.3 # m    
GRAV = 9.81 # m.s^2

start_ble_listener()

Q = np.array([1.0,0.0,0.0,0.0])
madgwick = Madgwick()


def estimate_wrist_position(q_rot):


    rot = R.from_quat([q_rot[1],q_rot[2],q_rot[3],q_rot[0]])

    dir = np.array([0,0,ARM_LENGTH])

    rotation = rot.apply(dir)



    return rotation

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
origin = np.array([0,0,0])
x,y,z = estimate_wrist_position(Q)
vector = np.array([x,y,z])

quiver = ax.quiver(*origin,*vector)

lims = [-1,1]
ax.set_xlim(lims)
ax.set_ylim(lims)
ax.set_zlim(lims)

def update(frame):
    global Q,quiver

    a = np.array(pose_data["a"]) * GRAV
    g = np.radians(np.array(pose_data["g"]))
    m = np.array(pose_data["m"])

    Q = madgwick.updateMARG(q=Q, acc=a, gyr=g,mag=m)
    roll,pitch,yaw = q2euler(Q)
    print(roll,pitch,yaw)

    quiver.remove()
    
    arm_vec = estimate_wrist_position(Q)

    quiver = ax.quiver(0, 0, 0, arm_vec[0], arm_vec[1], arm_vec[2], color='black', length=0.5, normalize=True,label='Arm')
    return quiver,

ani = FuncAnimation(fig,update,frames=100,interval=50,blit=False, repeat=True)

ax.quiver(0, 0, 0, 1, 0, 0, color='r', length=0.5, normalize=True, label='X-axis')
ax.quiver(0, 0, 0, 0, 1, 0, color='g', length=0.5, normalize=True, label='Y-axis')
ax.quiver(0, 0, 0, 0, 0, 1, color='b', length=0.5, normalize=True, label='Z-axis')


ax.legend()

plt.show()


