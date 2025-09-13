import tkinter as tk
import coppeliasim_zmqremoteapi_client as zmqRemoteApi
import time
import inverse_kinematics_solver4 as iks
# import inverse_kinematics_solver5 as iks

import numpy as np
import math
from math import pi as pi

# Connect client
client = zmqRemoteApi.RemoteAPIClient()
sim = client.getObject('sim')

# Joint handles
joint_names = ['/joint1', '/joint2', '/joint3', '/joint4', '/joint5', '/joint6']
joint_handles = [sim.getObject(joint) for joint in joint_names]

# Initial pose
pos = [-0.2, -0.2, 0.4]
rot = [0, 0, np.deg2rad(90)]
 
# Constants for UR5
a_dim = [0, 0.4251, 0.39215, 0, 0, 0]
d_dim = [0, 0, 0, 0.110, 0.09475, 0]
b = 0.0892
tp = 0.07495

# Initialize GUI
root = tk.Tk()
root.title("CoppeliaSim Joint Control")
sliders = {}

# Start simulation
sim.startSimulation()

# Get initial joint angles from CoppeliaSim
angles = []
for handle in joint_handles:
    angle = sim.getJointPosition(handle)  # Fixed: No unpacking
    angles.append(angle)
angles = np.array(angles)

def update_joint_angles():
    global angles, pos, rot
    # Get slider values
    new_pos = [sliders['x'].get() * 0.001, sliders['y'].get() * 0.001, sliders['z'].get() * 0.001]
    new_rot = [sliders['alpha'].get() * math.pi/180, sliders['beta'].get() * math.pi/180, sliders['gamma'].get() * math.pi/180]

    # Interpolate pose for smoother transitions
    steps = 5
    for i in range(steps + 1):
        t = i / steps
        interp_pos = [(1 - t) * p1 + t * p2 for p1, p2 in zip(pos, new_pos)]
        interp_rot = [(1 - t) * r1 + t * r2 for r1, r2 in zip(rot, new_rot)]
        
        # Compute IK solution
        new_angles = iks.choose_best_ik(angles, interp_pos, interp_rot, a_dim, d_dim, b, tp)
        
        # Log angles for debugging
        print("Joint angles:", [round(ang * 180/pi, 2) for ang in new_angles])
        
        # Update joint angles in CoppeliaSim
        for angle, handle in zip(new_angles, joint_handles):
            sim.setJointTargetPosition(handle, angle)
        
        # Update global angles
        angles = new_angles
        
        # Small delay
        time.sleep(0.01)
    
    # Update global pose
    pos[:] = new_pos
    rot[:] = new_rot
    
    root.after(100, update_joint_angles)

# Create sliders
position = ['x', 'y', 'z']
for name in position:
    tk.Label(root, text=f"{name} (m)").pack()
    sliders[name] = tk.Scale(root, from_=-1000, to=1000, orient="horizontal", resolution=1)
    if name == 'x':
        sliders[name].set(-200)
    elif name == 'y':
        sliders[name].set(-200)
    elif name == 'z':
        sliders[name].set(400)
    sliders[name].pack()

rotation = ['alpha', 'beta', 'gamma']
for name in rotation:
    tk.Label(root, text=f"{name} (deg)").pack()
    sliders[name] = tk.Scale(root, from_=-180, to=180, orient="horizontal", resolution=1)
    if name == 'alpha':
        sliders[name].set(0)
    elif name == 'beta':
        sliders[name].set(0)
    elif name == 'gamma':
        sliders[name].set(90)
    sliders[name].pack()

# Start updating joint angles
root.after(100, update_joint_angles)
root.mainloop() 

# Stop simulation
sim.stopSimulation()