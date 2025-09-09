import matplotlib.pyplot as plt
import numpy as np
import os


def get_unique_filename(directory, filename):
    """
    Returns a unique filename by appending (1), (2), ... if file exists.
    """
    base, ext = os.path.splitext(filename)
    counter = 1
    candidate = filename
    
    while os.path.exists(os.path.join(directory, candidate)):
        candidate = f"{base} ({counter}){ext}"
        counter += 1
    
    return os.path.join(directory, candidate)

def plot_joint_responses(times, positions, desired_angles,kp,ki,kd,method,save_dir="plots"):
    """
    Plot joint angles vs time for up to 6 joints in a 2x3 grid layout.
    """

    os.makedirs(save_dir, exist_ok=True)
    filename_suffix = f"Kp{kp}_Ki{ki}_Kd{kd}"
    num_joints = positions.shape[1]
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    axs = axs.flatten()  # flatten to 1D for easier indexing

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
        save_path = get_unique_filename(save_dir,"SERVO_joints.png")
    elif method == "SPEED":
        save_path = os.path.join(save_dir, f"{method}_joints_{filename_suffix}.png")
    else:
        return
    fig.savefig(save_path)
    plt.close(fig)  # close to free memory

    print(f"Saved plot to {save_path}")
    plt.show()
