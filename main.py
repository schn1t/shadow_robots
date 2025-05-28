from pose_listen import start_ble_listener, pose_data
from time import sleep

start_ble_listener()
n = 0
while True:
    if n % 4 == 0:
        roll = pose_data["roll"]
        pitch = pose_data["pitch"]
        accel = pose_data["a"][0]
        gyro = pose_data["g"]

        if roll is not None and pitch is not None:
            print(f"[Main] Roll: {roll:.2f}, Pitch: {pitch:.2f}")
            print(f"Angular acceleration: {accel}")
            print(f"Gyro velocity: {gyro}")
    else:
        print(n)
    n += 1
    sleep(0.5)
