#import evdev
from evdev import InputDevice, categorize, ecodes

# Find and create device
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    if 'Controller' in device.name:
        print('Found controller:', device.name)
        # Additional code for handling the controller can be added here
    print(device.path, device.name, device.phys)
    gamepad = InputDevice(device.path)
    

#prints out device info at start
print(gamepad)

#evdev takes care of polling the controller in a loop
for event in gamepad.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        print(categorize(event))