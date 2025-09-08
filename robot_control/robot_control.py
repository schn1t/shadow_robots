from wirelessMasterCallback import WirelessMasterCallback
from mtwManipulator import MtwManipulator
from pidController import PIDcontroller

import sys
import time

import numpy as np
import inverse_kinematics_solver4 as iks

from math import pi

import xsensdeviceapi as xda
from scipy.spatial.transform import Rotation as R


a_dim = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]  
d_dim = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
b = 0.0892 
tp = 0.07495 

angles = np.array([0.0, -pi/2, 0.0, -pi/2, 0.0, 0.0])

import rtde_control, rtde_receive
import time

def wrap_to_pi(angle):
    """Wrap angle(s) to [-pi, pi]. Accepts float or np.array."""
    return (angle + np.pi) % (2*np.pi) - np.pi

if __name__ == '__main__':

    ROBOT_IP = "127.0.0.1"
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    # Conservative gains (starting point). Tune per joint if needed.
    Kp = [6.0, 6.0, 6.0, 4.0, 3.0, 2.0]
    Ki = [0.5, 0.5, 0.5, 0.2, 0.2, 0.1]
    Kd = [2.0, 2.0, 2.5, 1.0, 0.8, 0.5]

    # Kp = [3.6, 3.6, 3.6, 2.4, 1.8, 1.2]
    # Ki = [0.05, 0.05, 0.05, 0.02, 0.02, 0.01]
    # Kd = [3.0, 3.0, 3.75, 1.5, 1.2, 0.75]

    pid = PIDcontroller(Kp=Kp, Ki=Ki, Kd=Kd, u_max=5.0, i_max=0.6, backcalc_beta=0.15)

    # initialize desired target with current pose to avoid jump
    q = np.array(rtde_r.getActualQ())
    q_des = q.copy()
    q_des_prev = q.copy()

    # timers
    last = time.perf_counter()
    stable_since = None
    
    shoulder_q = R.identity()
    elbow_q = R.identity()
    wrist_q = R.identity()

    desired_update_rate = 75
    desired_radio_channel = 19

    wireless_master_callback = WirelessMasterCallback()
    mtw_callbacks = []

    print("Constructing XsControl...")
    control = xda.XsControl.construct()
    if control is None:
        print("Failed to construct XsControl instance.")
        sys.exit(1)

    manipulator = MtwManipulator(control, wireless_master_callback, desired_update_rate, desired_radio_channel)


    deviceID_to_bodypart = {
        "00B4F130": "SHOU R",
        "00B4F1B4": "fARM R",
        "00B4F198": "HAND R"        
    }
    
    try:
        mtw_callbacks = manipulator.scanDevicesAndExtractData()
        quat_data = {idx: xda.XsQuaternion.identity() for idx in deviceID_to_bodypart.values()}
        euler_data = {idx: xda.XsEuler() for idx in deviceID_to_bodypart.values()}

        # This function checks for user input to break the loop
        def user_input_ready():
            return False  # Replace this with your method to detect user input

        while not user_input_ready():
            time.sleep(0)

            new_data_available = False
            for i in range(len(mtw_callbacks)):
                if mtw_callbacks[i].dataAvailable():
                    new_data_available = True
                    packet = mtw_callbacks[i].getOldestPacket()
                    mtw_deviceID = str(mtw_callbacks[i].device().deviceId())
                    bodypart_name = deviceID_to_bodypart.get(mtw_deviceID)
                    if bodypart_name:
                        quat_data[bodypart_name] = packet.orientationQuaternion()
                        euler_data[bodypart_name] = packet.orientationEuler()
                    mtw_callbacks[i].deleteOldestPacket()


            if new_data_available:
                # parse shoulder orientation
                shou_qW, shou_qX, shou_qY, shou_qZ = quat_data.get("SHOU R")
                shoulder_q = R.from_quat([-shou_qY, shou_qX, shou_qZ, shou_qW])
                # parse forearm orientation
                elbow_qW, elbow_qX, elbow_qY, elbow_qZ = quat_data.get("fARM R")
                elbow_q = R.from_quat([-elbow_qY, elbow_qX, elbow_qZ, elbow_qW])
                # parse hand orientation
                wrist_qW, wrist_qX, wrist_qY, wrist_qZ = quat_data.get("HAND R")
                wrist_q = R.from_quat([-wrist_qY, wrist_qX, wrist_qZ, wrist_qW])
                
                alpha = np.deg2rad(-180)
                beta = np.deg2rad(0)
                gamma = np.deg2rad(-90)

                R_sensor = R.from_quat([wrist_qZ, -wrist_qX, wrist_qY, wrist_qW]).as_matrix()

                # print(R_fixed)

                rot_matrix = np.rot90(R_sensor @ R.from_euler('zxy', [alpha, beta, gamma]).as_matrix(), k=-1)
                rot_matrix[:, [1, 2]] = rot_matrix[:, [2, 1]]
                rot_matrix[0][2] *= -1
                rot_matrix[1][2] *= -1
                rot_matrix[2][0] *= -1
                rot_matrix[2][1] *= -1

                # print(rot_matrix)

                L_upper   = 0.375   # shoulder → elbow
                L_forearm = 0.3   # elbow → wrist
                L_hand    = 0.075   # wrist → hand center

                local_upper = np.array([0.0, L_upper, 0.0])
                local_forearm = np.array([0.0, L_forearm, 0.0])
                local_hand = np.array([0.0, L_hand, 0.0])

                shoulder_pos = np.array([0.0, 0.0, 0.0])

                # Compute elbow position
                p_elbow = shoulder_pos + shoulder_q.apply(local_upper)
                # Compute wrist position
                p_wrist = p_elbow + elbow_q.apply(local_forearm)
                # Compute hand center position
                p_hand = p_wrist + wrist_q.apply(local_hand)

                scaled_pos = [
                    p_hand[0],  # Scale X
                    p_hand[1],  # Scale Y
                    p_hand[2]   # Scale Z
                ]

                now = time.perf_counter()
                loop_dt = now - last
                last = now
                if loop_dt <= 0.0 or loop_dt > 0.2:
                    loop_dt = 1/250  # guardrail if we had a hiccup

                # Read robot state
                q = np.array(rtde_r.getActualQ(), dtype=float)
                qd = np.array(rtde_r.getActualQd(), dtype=float)
                new_q_des = iks.choose_best_ik(angles, scaled_pos, rot_matrix, a_dim, d_dim, b, tp)
                angles = new_q_des
                q_real = np.deg2rad([0, -90, 0, -90, 0, 0])
                new_q_des += q_real

                new_q_des = np.array(new_q_des, dtype=float)

                def normalize_angle(angle):
                    return (angle - 2 * np.pi) % (4 * np.pi) - 2 * np.pi

                new_q_des = normalize_angle(new_q_des)
                
                dq_des_raw = wrap_to_pi(new_q_des - q_des_prev)
                max_step = 10.0 * loop_dt
                dq_des = np.clip(dq_des_raw, -max_step, max_step)
                q_des = wrap_to_pi(q_des_prev + dq_des)  # keep continuity
                q_des_prev = q_des.copy()

                # PID step -> desired joint velocity command
                u, e = pid.step(q_des, q, qd, loop_dt)

                # Safety: if comms or state look weird, bail
                if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
                    print("Non-finite in control; stopping.")
                    rtde_c.speedStop()
                    break

                # Command velocities
                # speedJ takes (qd, a, t). We set t=dt to make it time-consistent.
                rtde_c.speedJ(u.tolist(), 5.0, 1/250)

    except Exception as ex:  
        print(ex)
        print("****ABORT****")
        rtde_c.servoStop()
        rtde_c.disconnect()
        rtde_r.disconnect()
    except:
        print("An unknown fatal error has occurred. Aborting.")
        print("****ABORT****")
        rtde_c.servoStop()
        rtde_c.disconnect()
        rtde_r.disconnect()

    print("Closing XsControl...")
    control.close()

    print("Deleting mtw callbacks...")

    print("Successful exit.")
    print("Press [ENTER] to continue.")
    input()

    rtde_c.servoStop()
    rtde_c.disconnect()
    rtde_r.disconnect()
