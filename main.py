import asyncio
import struct
from bleak import BleakScanner
from bleak import BleakClient

address = "D166A5CD-7A51-51FF-77D1-9C87BFB6DBE6"
ROLL_UUID = "c24def16-a665-4da1-b87a-6706b03eb108"
PITCH_UUID = "c24def16-a665-4da1-b87a-6706b03eb109"

async def read_pose_data(address: str, roll_uuid: str, pitch_uuid: str):
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return None

        # Read raw bytes from the BLE characteristics
        roll_raw = await client.read_gatt_char(roll_uuid)
        pitch_raw = await client.read_gatt_char(pitch_uuid)

        # Unpack bytes into float values (assuming little-endian float format)
        roll = struct.unpack('<f', roll_raw)[0]
        pitch = struct.unpack('<f', pitch_raw)[0]

        return roll, pitch

roll,pitch = asyncio.run(read_pose_data(address,ROLL_UUID,PITCH_UUID))
print(roll,pitch)


# D166A5CD-7A51-51FF-77D1-9C87BFB6DBE6