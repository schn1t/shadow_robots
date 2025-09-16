import tkinter as tk
from tkinter import messagebox, ttk
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
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class UR5eGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("UR5e Robot Control")
        self.root.geometry("1200x600")

        self.robot_ip = "127.0.0.1"
        self.rtde_c = None
        self.rtde_r = None
        self.pid = None
        self.mtw_callbacks = None
        self.running = False

        # Robot parameters
        self.a_dim = [0.0, 0.4251, 0.39215, 0.0, 0.0, 0.0]
        self.d_dim = [0.0, 0.0, 0.0, 0.11000, 0.09475, 0.0]
        self.b = 0.0892
        self.tp = 0.07495
        self.angles = np.array([0.0, -pi/2, 0.0, -pi/2, 0.0, 0.0])
        self.Kp = 6.0
        self.Ki = 0.5
        self.Kd = 2.0
        self.speed_scale = tk.DoubleVar(value=1.0)

        # Plotting data
        self.times = []
        self.actual_positions = []
        self.desired_positions = []
        self.max_plot_points = 100  # Limit to prevent memory issues

        # GUI elements
        # Status Frame
        self.status_frame = tk.Frame(self.root)
        self.status_frame.pack(pady=10, side=tk.TOP, fill=tk.X)
        self.status_label = tk.Label(self.status_frame, text="Robot Status: Disconnected", fg="red")
        self.status_label.pack()

        # Joint Angles and TCP Pose
        self.joint_label = tk.Label(self.status_frame, text="Joint Angles: N/A")
        self.joint_label.pack()
        self.tcp_label = tk.Label(self.status_frame, text="TCP Pose: N/A")
        self.tcp_label.pack()

        # Control Buttons
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(pady=10, side=tk.TOP, fill=tk.X)
        self.connect_button = tk.Button(self.control_frame, text="Connect Robot", command=self.connect_robot)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        self.start_button = tk.Button(self.control_frame, text="Start Control", command=self.start_control, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT, padx=5)
        self.stop_button = tk.Button(self.control_frame, text="Stop Control", command=self.stop_control, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        self.emergency_button = tk.Button(self.control_frame, text="Emergency Stop", command=self.emergency_stop, bg="red", fg="white")
        self.emergency_button.pack(side=tk.LEFT, padx=5)

        # Speed Scale
        self.speed_frame = tk.Frame(self.root)
        self.speed_frame.pack(pady=5, side=tk.TOP, fill=tk.X)
        tk.Label(self.speed_frame, text="Speed Scale:").pack(side=tk.LEFT)
        tk.Scale(self.speed_frame, from_=0.1, to=2.0, resolution=0.1, orient=tk.HORIZONTAL, variable=self.speed_scale).pack(side=tk.LEFT)

        # Plotting Frame
        self.plot_frame = tk.Frame(self.root)
        self.plot_frame.pack(pady=10, side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig, self.axs = plt.subplots(2, 3, figsize=(8, 4))
        self.axs = self.axs.flatten()
        for j in range(6):
            self.axs[j].set_title(f'Joint {j+1}')
            self.axs[j].set_xlabel('Time (s)')
            self.axs[j].set_ylabel('Position (rad)')
            self.axs[j].grid(True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plt.tight_layout()

        # Log Window
        self.log_frame = tk.Frame(self.root)
        self.log_frame.pack(pady=10, side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(self.log_frame, height=8, width=40)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = ttk.Scrollbar(self.log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)

        # Exit Button
        self.exit_button = tk.Button(self.root, text="Exit", command=self.exit_program)
        self.exit_button.pack(pady=5, side=tk.TOP)

        # Start status updates
        self.update_status()

    def log_message(self, message):
        self.log_text.insert(tk.END, f"{time.strftime('%H:%M:%S')}: {message}\n")
        self.log_text.see(tk.END)

    def update_status(self):
        if self.rtde_r and self.running:
            q = self.rtde_r.getActualQ()
            tcp = self.rtde_r.getActualTCPPose()
            self.joint_label.config(text=f"Joint Angles: {np.round(q, 3)}")
            self.tcp_label.config(text=f"TCP Pose: {np.round(tcp, 3)}")
        self.root.after(100, self.update_status)

    def update_plot(self, time_val, actual_q, desired_q):
        self.times.append(time_val)
        self.actual_positions.append(actual_q.copy())
        self.desired_positions.append(desired_q.copy())

        if len(self.times) > self.max_plot_points:
            self.times.pop(0)
            self.actual_positions.pop(0)
            self.desired_positions.pop(0)

        for ax in self.axs:
            ax.clear()
            ax.set_title(ax.get_title())
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Position (rad)')
            ax.grid(True)

        for j in range(6):
            times = np.array(self.times) - self.times[0] if self.times else np.array([])
            actual = np.array(self.actual_positions)[:, j] if self.actual_positions else np.array([])
            desired = np.array(self.desired_positions)[:, j] if self.desired_positions else np.array([])
            self.axs[j].plot(times, actual, label='Actual')
            self.axs[j].plot(times, desired, 'r--', label='Desired')
            self.axs[j].legend()

        self.canvas.draw()

    def connect_robot(self):
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.status_label.config(text="Robot Status: Connected", fg="green")
            self.connect_button.config(state=tk.DISABLED)
            self.start_button.config(state=tk.NORMAL)
            self.emergency_button.config(state=tk.NORMAL)
            self.log_message("Connected to UR5e robot.")
        except Exception as e:
            self.log_message(f"Connection failed: {str(e)}")
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
        self.emergency_button.config(state=tk.DISABLED)
        self.log_message("Robot disconnected.")

    def start_control(self):
        if not self.rtde_c or not self.rtde_r:
            messagebox.showerror("Error", "Robot not connected.")
            self.log_message("Start failed: Robot not connected.")
            return
        self.running = True
        self.times = []
        self.actual_positions = []
        self.desired_positions = []
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.emergency_button.config(state=tk.NORMAL)
        self.log_message("Control loop started.")
        self.control_loop()

    def stop_control(self):
        self.running = False
        if self.rtde_c:
            self.rtde_c.speedStop()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.log_message("Control loop stopped.")

    def emergency_stop(self):
        self.running = False
        if self.rtde_c:
            self.rtde_c.servoStop()
        if self.pid:
            self.pid.reset()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.emergency_button.config(state=tk.NORMAL)
        self.log_message("Emergency stop triggered. PID reset.")
        messagebox.showinfo("Emergency Stop", "Robot stopped and PID reset.")

    def exit_program(self):
        self.stop_control()
        self.disconnect_robot()
        plt.close(self.fig)
        self.root.quit()

    def control_loop(self):
        self.pid = PIDcontroller(Kp=[self.Kp]*6, Ki=[self.Ki]*6, Kd=[self.Kd]*6, u_max=5.0, i_max=0.6, backcalc_beta=0.15)
        q = np.array(self.rtde_r.getActualQ())
        q_des = q.copy()
        q_des_prev = q.copy()
        start_time = time.perf_counter()
        last = start_time
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
            self.mtw_callbacks = manipulator.scanDevicesAndExtractData()
            quat_data = {idx: xda.XsQuaternion.identity() for idx in deviceID_to_bodypart.values()}
            self.log_message("Xsens devices initialized.")
            while self.running:
                new_data_available = False
                for i in range(len(self.mtw_callbacks)):
                    if self.mtw_callbacks[i].dataAvailable():
                        new_data_available = True
                        packet = self.mtw_callbacks[i].getOldestPacket()
                        mtw_deviceID = str(self.mtw_callbacks[i].device().deviceId())
                        bodypart_name = deviceID_to_bodypart.get(mtw_deviceID)
                        if bodypart_name:
                            quat_data[bodypart_name] = packet.orientationQuaternion()
                        self.mtw_callbacks[i].deleteOldestPacket()
                if new_data_available:
                    self.log_message("New Xsens data received.")
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
                    new_q_des = (new_q_des - 2 * pi) % (4 * pi) - 2 * pi
                    dq_des_raw = (new_q_des - q_des_prev + pi) % (2 * pi) - pi
                    max_step = 10.0 * loop_dt
                    dq_des = np.clip(dq_des_raw, -max_step, max_step)
                    q_des = (q_des_prev + dq_des + pi) % (2 * pi) - pi
                    q_des_prev = q_des.copy()

                    u, e = self.pid.step(q_des, q, qd, loop_dt)
                    if not np.all(np.isfinite(u)) or not np.all(np.isfinite(e)):
                        self.log_message("Non-finite control values; stopping.")
                        messagebox.showerror("Error", "Non-finite control values; stopping.")
                        self.emergency_stop()
                        break
                    u_scaled = u * self.speed_scale.get()
                    self.rtde_c.speedJ(u_scaled.tolist(), 5.0, 1/250)

                    # Update plot with current time and positions
                    self.update_plot(now - start_time, q, q_des)

                self.root.update()
                time.sleep(0.01)
        except Exception as e:
            self.log_message(f"Control loop error: {str(e)}")
            messagebox.showerror("Error", f"Control loop error: {str(e)}")
            self.emergency_stop()
        finally:
            control.close()
            self.log_message("Xsens control closed.")

if __name__ == "__main__":
    root = tk.Tk()
    app = UR5eGUI(root)
    root.mainloop()