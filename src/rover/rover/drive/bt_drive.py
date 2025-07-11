import asyncio
from bleak import BleakScanner

async def find_pro_controller_info():
    print("Scanning for Bluetooth devices...")
    devices = await BleakScanner.discover(timeout=10.0) # Scan for 10 seconds

    pro_controller_info = None
    for device in devices:
        print(f"Found device: {device.name} ({device.address})")
        # The name is usually "Pro Controller"
        if device.name and "Pro Controller" in device.name:
            print(f"Found potential Pro Controller: {device.name} ({device.address})")
            pro_controller_info = device
            break

    if pro_controller_info:
        print(f"\nPro Controller found: Name='{pro_controller_info.name}', Address='{pro_controller_info.address}'")
        print("Use this address for connecting.")
        return pro_controller_info.address
    else:
        print("Nintendo Switch Pro Controller not found during scan.")
        return None

async def main_discovery():
    pro_controller_id = await find_pro_controller_info()
    if pro_controller_id:
        print(f"You can now try connecting to: {pro_controller_id}")
        # You can then pass this ID to your connect_by_mac_address function
        # await connect_by_mac_address(pro_controller_id) # Call your connection function

if __name__ == "__main__":
    asyncio.run(main_discovery())