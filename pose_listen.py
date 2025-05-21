import asyncio
import threading
import struct
from bleak import BleakClient
from time import sleep

address = "D166A5CD-7A51-51FF-77D1-9C87BFB6DBE6"
ROLL_UUID = "c24def16-a665-4da1-b87a-6706b03eb108"
PITCH_UUID = "c24def16-a665-4da1-b87a-6706b03eb109"

pose_data = {"roll":None,"pitch":None}
lock = threading.Lock()


def roll_callback(sender:int,data:bytearray):
    roll = struct.unpack('<f',data)[0]
    pose_data["roll"] = roll
   # print(f"Roll: {roll:.2f}")

def pitch_callback(sender:int,data:bytearray):
    pitch = struct.unpack('<f',data)[0]
    pose_data["pitch"] = pitch
   # print(f"Pitch: {pitch:.2f}")

async def poll_angles():
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return
        await client.start_notify(ROLL_UUID,roll_callback)
        await client.start_notify(PITCH_UUID,pitch_callback)
        print("Listening for notifications... Press Ctrl+C to stop.")
        while True:
            await asyncio.sleep(1)  # Keep the program alive

def start_ble_loop():
    asyncio.run(poll_angles())

def start_ble_listener():
    thread = threading.Thread(target=start_ble_loop,daemon=True)
    thread.start()
    print("BLE listener started in background")