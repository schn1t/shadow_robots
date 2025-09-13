import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler
import threading
import time
import math
from math import pi as pi

# Hardware imports (from robot_control.py)
try:
    import rtde_control
    import rtde_receive
    from pidController import PIDcontroller
    import inverse_kinematics_solver4 as iks
    from pose_listen import start_ble_listener, pose_data
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("Hardware libs not available; sim mode only.")

# Sim imports (from updated_code.py)
try:
    import coppeliasim_zmqremoteapi_client as zmqRemoteApi
    SIM_AVAILABLE = True
except ImportError:
    SIM_AVAILABLE = False
    print("CoppeliaSim not available; hardware mode only.")

# Plotting (from plotting.py)
def get_unique_filename(directory, filename):
    import os
    base, ext = os.path.splitext(filename)
    counter = 1
    candidate = filename
    while os.path.exists(os.path.join(directory, candidate)):
        candidate = f"{base} ({counter}){ext}"
        counter += 1
    return os.path.join(directory, candidate)

def plot_joint_responses(times, positions, desired_angles, kp, ki, kd, method, save_dir="plots"):
    import os
    os.makedirs(save_dir, exist_ok=True)
    filename_suffix = f"Kp{kp}_Ki{ki}_Kd{kd}"
    num_joints = positions.shape[1]
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    axs = axs.flatten()
    for j in range(num_joints):
        axs[j].plot(times, positions[:, j], label='Actual Position')
        axs[j].axhline(desired_angles[j], color='r', linestyle='--', label='Desired Position')
        axs[j].set_title(f'Joint {j+1}')
        axs[j].set_xlabel('Time (s)')
        axs[j].set_ylabel('Position (rad)')
        axs[j].legend()
        axs[j].grid(True)
    plt.tight_layout()
    if method == "SERVO":
        save_path = get_unique_filename(save_dir, "SERVO_joints.png")
    elif method == "SPEED":
        save_path = os.path.join(save_dir, f"{method}_joints_{filename_suffix}.png")
    else:
        return
    fig.savefig(save_path)
    plt.close(fig)
    print(f"Saved plot to {save_path}")

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Shadow Robot Control GUI - Ingenuity Demo")
        self.mode = tk.StringVar(value="Sim")  # Default to Sim
        self.arms = [None]  # Single arm for now; extend to [None, None] for two
        self.running = [False]
        self.gains = [{"kp": [6.0]*6, "ki": [0.5]*6, "kd": [2.0]*6}]  # Per-joint from robot_control.py
        self.limits = [{"max_rot": np.pi/2, "max_speed": 1.0}]  # Rad/s
        self.sensors = ["ROLL", "PITCH", "GYRO", "ACCEL", "MAG"]  # From pose_listen.py UUIDs
        self.sensor_assignments = {"Arm1": self.sensors[:3]}  # Mock: first 3 for arm
        self.ur_dims = {"a": [0, 0.4251, 0.39215, 0, 0, 0], "d": [0, 0, 0, 0.110, 0.09475, 0], "b": 0.0892, "tp": 0.07495}
        
        # Init hardware/sim
        if HARDWARE_AVAILABLE:
            self.rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")  # Update IP
            self.rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
            self.pid = PIDcontroller(Kp=self.gains[0]["kp"], Ki=self.gains[0]["ki"], Kd=self.gains[0]["kd"])
        if SIM_AVAILABLE:
            self.client = zmqRemoteApi.RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.joint_handles = [self.sim.getObject(f'/joint{i+1}') for i in range(6)]
            self.sim.startSimulation()
        
        # Start BLE
        if HARDWARE_AVAILABLE:
            start_ble_listener()
            self.Q = np.array([1.0, 0.0, 0.0, 0.0])
            self.madgwick = Madgwick()
            self.arm_length = 0.3
        
        # Create notebook
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(pady=10, expand=True, fill=tk.BOTH)
        
        # Tabs
        self.create_mode_tab("Mode Selector")
        self.create_arm_tab("Arm 1 Control", 0)
        self.create_imu_tab("IMU Monitor")
        
        # Update loop
        self.update_loop()
    
    def create_mode_tab(self, name):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text=name)
        ttk.Label(frame, text="Control Mode:").pack(pady=10)
        ttk.Radiobutton(frame, text="Simulation (CoppeliaSim)", variable=self.mode, value="Sim").pack()
        ttk.Radiobutton(frame, text="Hardware (UR5e)", variable=self.mode, value="Hardware", state="normal" if HARDWARE_AVAILABLE else "disabled").pack()
        ttk.Button(frame, text="Apply Mode", command=self.apply_mode).pack(pady=10)
    
    def create_arm_tab(self, name, arm_idx):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text=name)
        
        # PID Sliders (per-joint; simplified to average for demo)
        ttk.Label(frame, text="PID Gains (Average)").grid(row=0, column=0, columnspan=2, pady=5)
        for i, param in enumerate(["kp", "ki", "kd"]):
            ttk.Label(frame, text=param.upper()).grid(row=i+1, column=0, padx=5)
            slider = ttk.Scale(frame, from_=0.0, to=10.0, orient=tk.HORIZONTAL,
                               command=lambda x, p=param, idx=arm_idx: self.update_gain(p, float(x), idx))
            slider.set(np.mean(self.gains[arm_idx][param]))
            slider.grid(row=i+1, column=1, padx=5, pady=2)
        
        # Safety Limits
        ttk.Label(frame, text="Safety Limits").grid(row=4, column=0, columnspan=2, pady=5)
        ttk.Label(frame, text="Max Rotation (rad)").grid(row=5, column=0)
        self.max_rot_var = tk.StringVar(value=str(self.limits[arm_idx]["max_rot"]))
        ttk.Entry(frame, textvariable=self.max_rot_var, width=10).grid(row=5, column=1, padx=5)
        ttk.Label(frame, text="Max Speed (m/s)").grid(row=6, column=0)
        self.max_speed_var = tk.StringVar(value=str(self.limits[arm_idx]["max_speed"]))
        ttk.Entry(frame, textvariable=self.max_speed_var, width=10).grid(row=6, column=1, padx=5)
        ttk.Button(frame, text="Update Limits", command=lambda: self.update_limit(arm_idx)).grid(row=7, column=0, columnspan=2)
        
        # Sensor Assignment
        ttk.Label(frame, text="Assign Sensors").grid(row=8, column=0, columnspan=2, pady=5)
        sensor_listbox = tk.Listbox(frame, selectmode=tk.MULTIPLE, height=5)
        for s in self.sensors:
            sensor_listbox.insert(tk.END, s)
        sensor_listbox.grid(row=9, column=0, columnspan=2)
        ttk.Button(frame, text="Assign to Arm", command=lambda idx=arm_idx: self.assign_sensors_from_list(sensor_listbox, idx)).grid(row=10, column=0, columnspan=2)
        
        # Controls
        ttk.Button(frame, text="Start", command=lambda: self.start_arm(arm_idx)).grid(row=11, column=0, pady=10)
        ttk.Button(frame, text="Stop", command=lambda: self.stop_arm(arm_idx)).grid(row=11, column=1, pady=10)
        ttk.Button(frame, text="Test PID", command=lambda: self.test_pid(arm_idx)).grid(row=12, column=0, columnspan=2)
        
        # Status
        self.status_label = ttk.Label(frame, text="Status: Stopped")
        self.status_label.grid(row=13, column=0, columnspan=2, pady=5)
    
    def create_imu_tab(self, name):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text=name)
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))
        self.canvas = FigureCanvasTkAgg(self.fig, frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.quiver = None
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-1,1])
        self.ax.set_zlim([-1,1])
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
    
    def apply_mode(self):
        print(f"Switched to {self.mode.get()} mode")
        # Re-init if needed (e.g., restart sim)
    
    def update_gain(self, param, value, arm_idx):
        self.gains[arm_idx][param] = [value] * 6  # Apply to all joints
        if HARDWARE_AVAILABLE and self.mode.get() == "Hardware":
            self.pid = PIDcontroller(Kp=self.gains[arm_idx]["kp"], Ki=self.gains[arm_idx]["ki"], Kd=self.gains[arm_idx]["kd"])
        print(f"Updated {param} to {value}")
    
    def update_limit(self, arm_idx):
        self.limits[arm_idx]["max_rot"] = float(self.max_rot_var.get())
        self.limits[arm_idx]["max_speed"] = float(self.max_speed_var.get())
        print("Limits updated")
    
    def assign_sensors_from_list(self, listbox, arm_idx):
        selected = [self.sensors[listbox.curselection()[i]] for i in range(listbox.curselection().__len__())]
        self.sensor_assignments["Arm1"] = selected
        print(f"Assigned sensors: {selected}")
    
    def start_arm(self, arm_idx):
        self.running[arm_idx] = True
        self.status_label.config(text="Status: Running")
        if self.mode.get() == "Sim" and SIM_AVAILABLE:
            self.sim.startSimulation()
    
    def stop_arm(self, arm_idx):
        self.running[arm_idx] = False
        self.status_label.config(text="Status: Stopped")
        if self.mode.get() == "Hardware" and HARDWARE_AVAILABLE:
            self.rtde_c.servoStop()
        elif self.mode.get() == "Sim" and SIM_AVAILABLE:
            self.sim.stopSimulation()
    
    def test_pid(self, arm_idx):
        # Mock test: Run a sine trajectory for 5s, record positions
        if not HARDWARE_AVAILABLE and self.mode.get() == "Hardware":
            return
        start_time = time.time()
        times, actual_pos, des_pos = [], [], []
        while time.time() - start_time < 5:
            t = time.time() - start_time
            times.append(t)
            des = np.array([pi/4 * np.sin(2*pi*t) for _ in range(6)])  # Sine wave
            actual = des * 0.95 + np.random.normal(0, 0.05, 6)  # Mock response
            actual_pos.append(actual)
            des_pos.append(des)
            time.sleep(0.05)
        actual_pos = np.array(actual_pos)
        des_pos = np.array(des_pos)
        plot_joint_responses(np.array(times), actual_pos, des_pos[0],  # Use first des for plot
                             np.mean(self.gains[arm_idx]["kp"]), np.mean(self.gains[arm_idx]["ki"]), np.mean(self.gains[arm_idx]["kd"]),
                             "SPEED" if self.mode.get() == "Hardware" else "SERVO")
    
    def update_loop(self):
        if self.running[0] and HARDWARE_AVAILABLE and self.mode.get() == "Hardware":
            q = np.array(self.rtde_r.getActualQ())
            qd = np.array(self.rtde_r.getActualQd())
            # Process IMU to pose (Madgwick)
            a = np.array(pose_data["a"])
            g = np.radians(np.array(pose_data["g"]))
            m = np.array(pose_data["m"])
            self.Q = self.madgwick.updateMARG(q=self.Q, acc=a, gyr=g, mag=m)
            # Estimate end-effector pos/rot from quaternion (simplified)
            rot = R.from_quat([self.Q[1], self.Q[2], self.Q[3], self.Q[0]])
            pos = rot.apply([0, 0, self.arm_length]) + [0.4, 0, 0]  # Offset
            eul = q2euler(self.Q)
            # IK
            q_des = iks.choose_best_ik(q, pos, eul, **self.ur_dims)
            q_des = np.clip(q_des, -self.limits[0]["max_rot"], self.limits[0]["max_rot"])
            # PID
            u, e = self.pid.step(q_des, q, qd, 0.004)  # 250Hz dt
            self.rtde_c.speedJ(u.tolist(), 5.0, 0.004)
            self.status_label.config(text=f"Status: Running | EE Pos: {pos[:2]:.2f}")
        elif self.running[0] and SIM_AVAILABLE and self.mode.get() == "Sim":
            # From updated_code.py: Use sliders for pos/rot (add sliders to tab if needed)
            pass  # Placeholder: Call your IK/setJointTargetPosition here
        
        # IMU plot update
        if HARDWARE_AVAILABLE:
            self.update_imu_plot()
        
        self.root.after(50, self.update_loop)  # 20Hz
    
    def update_imu_plot(self):
        if self.quiver:
            self.quiver.remove()
        a = np.array(pose_data["a"])
        g = np.radians(np.array(pose_data["g"]))
        m = np.array(pose_data["m"])
        self.Q = self.madgwick.updateMARG(q=self.Q, acc=a, gyr=g, mag=m)
        arm_vec = R.from_quat([self.Q[1], self.Q[2], self.Q[3], self.Q[0]]).apply([0, 0, self.arm_length])
        self.quiver = self.ax.quiver(0, 0, 0, *arm_vec, color='black', length=0.5)
        self.canvas.draw()
    
    def run(self):
        self.root.mainloop()
        if SIM_AVAILABLE:
            self.sim.stopSimulation()
        if HARDWARE_AVAILABLE:
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    app.run()