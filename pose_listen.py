import asyncio
import threading
import struct
from bleak import BleakClient
from time import sleep

address = "D166A5CD-7A51-51FF-77D1-9C87BFB6DBE6"
ROLL_UUID = "c24def16-a665-4da1-b87a-6706b03eb108"
PITCH_UUID = "c24def16-a665-4da1-b87a-6706b03eb109"
G_UUID = "c24def16-a665-4da1-b87a-6706b03eb110"
A_UUID = "c24def16-a665-4da1-b87a-6706b03eb111"
M_UUID = "c24def16-a665-4da1-b87a-6706b03eb112"

pose_data = {"roll":0,"pitch":0,"g":[0,0,0],"a":[0,0,0],"m":[0,0,0]}
lock = threading.Lock()


def roll_callback(sender:int,data:bytearray):
    roll = struct.unpack('<f',data)[0]
    pose_data["roll"] = roll
   # print(f"Roll: {roll:.2f}")

def pitch_callback(sender:int,data:bytearray):
    pitch = struct.unpack('<f',data)[0]
    pose_data["pitch"] = pitch
   # print(f"Pitch: {pitch:.2f}")

def vel_callback(sender: int, data: bytearray):
    # Unpack 3 little-endian floats from the 12-byte data
    wx, wy, wz = struct.unpack('<fff', data)
    pose_data["g"] = [wx, wy, wz]
    #print(f"Angular Velocity -> wx: {wx:.3f}, wy: {wy:.3f}, wz: {wz:.3f}")
    return (wx, wy, wz)

def accel_callback(sender: int, data: bytearray):
    ax, ay, az = struct.unpack('<fff', data)
    pose_data["a"] = [ax, ay, az]
    #print(f"Acceleration -> ax: {ax:.3f}, ay: {ay:.3f}, az: {az:.3f}")
    return (ax, ay, az)

def mag_callback(sender: int, data: bytearray):
    mx, my, mz = struct.unpack('<fff', data)
    pose_data["m"] = [mx, my, mz]

    return (mx,my,mz)

async def poll_angles():
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return
        await client.start_notify(ROLL_UUID,roll_callback)
        await client.start_notify(PITCH_UUID,pitch_callback)
        await client.start_notify(G_UUID,vel_callback)
        await client.start_notify(A_UUID,accel_callback)
        await client.start_notify(M_UUID,mag_callback)
        print("Listening for notifications... Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)  # Keep the program alive

def start_ble_loop():
    asyncio.run(poll_angles())

def start_ble_listener():
    thread = threading.Thread(target=start_ble_loop,daemon=True)
    thread.start()
    print("BLE listener started in background")