def roll_callback(sender:int,data:bytearray):
    roll = struct.unpack('<f',data)[0]
    pose_data["roll"] = roll
    print(f"Roll: {roll:.2f}")

def pitch_callback(sender:int,data:bytearray):
    pitch = struct.unpack('<f',data)[0]
    pose_data["pitch"] = pitch
    print(f"Pitch: {pitch:.2f}")

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


asyncio.run(poll_angles())
n = 1
while True:
    print(n)
    n += 1
    sleep(0.5)

# async def read_pose_data():
#     async with BleakClient(address) as client:
#         if not client.is_connected:
#             print("Failed to connect.")
#             return
#         while True:
#             try:
#                 # Read raw bytes from the BLE characteristics
#                 roll_raw = await client.read_gatt_char(ROLL_UUID)
#                 pitch_raw = await client.read_gatt_char(PITCH_UUID)

#                 # Unpack bytes into float values (assuming little-endian float format)
#                 roll = struct.unpack('<f', roll_raw)[0]
#                 pitch = struct.unpack('<f', pitch_raw)[0]
#                 with lock:
#                     pose_data["roll"] = roll
#                     pose_data["pitch"] = pitch
#             except Exception as e:
#                 print("Error reading pose data: ",e)

#         return roll, pitch
    
# while True:
#     roll,pitch = asyncio.run(read_pose_data(address,ROLL_UUID,PITCH_UUID))
#     print(roll,pitch)
#     sleep(0.1)

# D166A5CD-7A51-51FF-77D1-9C87BFB6DBE6