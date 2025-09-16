import tkinter as tk
from tkinter import messagebox
import sys
import time
import numpy as np
import rtde_control
import rtde_receive
from pidController import PIDcontroller
from mtwManipulator import MtwManipulator
from wirelessMasterCallback import WirelessMasterCallback
import xsensdeviceapi as xda
from scipy.spatial.transform import Rotation as R
import inverse_kinematics_solver4 as iks
from math import pi

class UR5eGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UR5e Robot Control")
        self.root.geometry("400x300")

        self.robot_ip = "127.0.0.1"
        self.rtde_c = None
        self.rtde_r = None
        self.control_thread = None
        self.running = False

        # Robot parameters
        self.a_dim = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]
        self.d_dim = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
        self.b = 0.0892
        self.tp = 0.07495
        self.angles = np.array([0.0, -pi/2, 0.0, -pi/2, 0.0, 0.0])
        self.Kp = [6.0, 6.0, 6.0, 4.0, 3.0, 2.0]
        self.Ki = [0.5, 0.5, 0.5, 0.2, 0.2, 0.1]
        self.Kd = [2.0, 2.0, 2.5, 1.0, 0.8, 0.5]

        # GUI elements
        self.status_label = tk.Label(root, text="Robot Status: Disconnected", fg="red")
        self.status_label.pack(pady=10)

        self.connect_button = tk.Button(root, text="Connect Robot", command=self.connect_robot)
        self.connect_button.pack(pady=5)

        self.start_button = tk.Button(root, text="Start Control", command=self.start_control, state=tk.DISABLED)
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(root, text="Stop Control", command=self.stop_control, state=tk.DISABLED)
        self.stop_button.pack(pady=5)

        self.exit_button = tk.Button(root, text="Exit", command=self.exit_program)
        self.exit_button.pack(pady=5)

    def connect_robot(self):
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.status_label.config(text="Robot Status: Connected", fg="green")
            self.connect_button.config(state=tk.DISABLED)
            self.start_button.config(state=tk.NORMAL)
            messagebox.showinfo("Success", "Connected to UR5e robot.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {str(e)}")

    def disconnect_robot(self):
        if self.rtde_c:
            self.rtde_c.servoStop()
            self.rtde_c.disconnect()
        if self.rtde_r:
            self.rtde_r.disconnect()
        self.rtde_c = None
        self.rtde_r = None
        self.status_label.config(text="Robot Status: Disconnected", fg="red")
        self.connect_button.config(state=tk.NORMAL)
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)

    def start_control(self):
        if not self.rtde_c or not self.rtde_r:
            messagebox.showerror("Error", "Robot not connected.")
            return
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.control_loop()

    def stop_control(self):
        self.running = False
        if self.rtde_c:
            self.rtde_c.speedStop()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def exit_program(self):
        self.stop_control()
        self.disconnect_robot()
        self.root.quit()

    def control_loop(self):
        pid = PIDcontroller(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, u_max=5.0, i_max=0.6, backcalc_beta=0.15)
        q = np.array(self.rtde_r.getActualQ())
        q_des = q.copy()
        q_des_prev = q.copy()
        last = time.perf_counter()
        shoulder_q = R.identity()
        elbow_q = R.identity()
        wrist_q = R.identity()
        desired_update_rate = 75
        desired_radio_channel = 19
        wireless_master_callback = WirelessMasterCallback()
        control = xda.XsControl.construct()
        manipulator = MtwManipulator(control, wireless_master_callback, desired_update_rate, desired_radio_channel)
        deviceID_to_bodypart = {
            "00B4F130": "SHOU R",
            "00B4F1B4": "fARM R",
            "00B4F198": "HAND R"
        }

        try:
            mtw_callbacks = manipulator.scanDevicesAndExtractData()
            quat_data = {idx: xda.XsQuaternion.identity() for idx in deviceID_to_bodypart.values()}
            while self.running:
                new_data_available = False
                for i in range(len(mtw_callbacks)):
                    if mtw_callbacks[i].dataAvailable():
                        new_data_available = True
                        packet = mtw_callbacks[i].getOldestPacket()
                        mtw_deviceID = str(mtw_callbacks[i].device().deviceId())
                        bodypart_name = deviceID_to_bodypart.get(mtw_deviceID)
                        if bodypart_name:
                            quat_data[bodypart_name] = packet.orientationQuaternion()
                        mtw_callbacks[i].deleteOldestPacket()

                if new_data_available:
                    shou_qW, shou_qX, shou_qY, shou_qZ = quat_data.get("SHOU R")
                    shoulder_q = R.from_quat([-shou_qY, shou_qX, shou_qZ, shou_qW])
                    elbow_qW, elbow_qX, elbow_qY, elbow_qZ = quat_data.get("fARM R")
                    elbow_q = R.from_quat([-elbow_qY, elbow_qX, elbow_qZ, elbow_qW])
                    wrist_qW, wrist_qX, wrist_qY, wrist_qZ = quat_data.get("HAND R")
                    wrist_q = R.from_quat([-wrist_qY, wrist_qX, wrist_qZ, wrist_qW])

                    alpha = np.deg2rad(-180)
                    beta = np.deg2rad(0)
                    gamma = np.deg2rad(-90)
                    R_sensor = R.from_quat([wrist_qZ, -wrist_qX, wrist_qY, wrist_qW]).as_matrix()
                    rot_matrix = np.rot90(R_sensor @ R.from_euler('zxy', [alpha, beta, gamma]).as_matrix(), k=-1)
                    rot_matrix[:, [1, 2]] = rot_matrix[:, [2, 1]]
                    rot_matrix[0][2] *= -1
                    rot_matrix[1][2] *= -1
                    rot_matrix[2][0] *= -1
                    rot_matrix[2][1] *= -1

                    L_upper = 0.375
                    L_forearm = 0.3
                    L_hand = 0.075
                    shoulder_pos = np.array([0.0, 0.0, 0.0])
                    p_elbow = shoulder_pos + shoulder_q.apply([0.0, L_upper, 0.0])
                    p_wrist = p_elbow + elbow_q.apply([0.0, L_forearm, 0.0])
                    p_hand = p_wrist + wrist_q.apply([0.0, L_hand, 0.0])
                    scaled_pos = [p_hand[0], p_hand[1], p_hand[2]]

                    now = time.perf_counter()
                    loop_dt = now - last
                    last = now
                    if loop_dt <= 0.0 or loop_dt > 0.2:
                        loop_dt = 1/250

                    q = np.array(self.rtde_r.getActualQ(), dtype=float)
                    qd = np.array(self.rtde_r.getActualQd(), dtype=float)
                    new_q_des = iks.choose_best_ik(self.angles, scaled_pos, rot_matrix, self.a_dim, self.d_dim, self.b, self.tp)
                    self.angles = new_q_des
                    q_real = np.deg2rad([0, -90, 0, -90, 0, 0])
                    new_q_des += q_real
                    new_q_des = np.array(new_q_des, dtype=float)
                    new_q_des = (new_q_des - 2 * np.pi) % (4 * np.pi) - 2 * np.pi
                    dq_des_raw = (new_q_des - q_des_prev + np.pi) % (2 * np.pi) - np.pi
                    max_step = 10.0 * loop_dt
                    dq_des = np.clip(dq_des_raw, -max_step, max_step)
                    q_des = (q_des_prev + dq_des + np.pi) % (2 * np.pi) - np.pi
                    q_des_prev = q_des.copy()

                    u, e = pid.step(q_des, q, qd, loop_dt)
                    if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
                        messagebox.showerror("Error", "Non-finite control values; stopping.")
                        self.stop_control()
                        break
                    self.rtde_c.speedJ(u.tolist(), 5.0, 1/250)
                self.root.update()
                time.sleep(0.01)
        except Exception as e:
            messagebox.showerror("Error", f"Control loop error: {str(e)}")
            self.stop_control()
        finally:
            control.close()

if __name__ == "__main__":
    root = tk.Tk()
    app = UR5eGUI(root)
    root.mainloop()