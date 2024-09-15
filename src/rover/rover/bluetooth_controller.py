#import evdev
from evdev import InputDevice, categorize, ecodes
import evdev

CENTER_TOLERANCE = 350
STICK_MAX = 65536
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

def get_stick_value(raw_value):
    if raw_value > CENTER_TOLERANCE:
        return (raw_value - CENTER_TOLERANCE) / (STICK_MAX - CENTER_TOLERANCE)
    elif raw_value < -CENTER_TOLERANCE:
        return (raw_value + CENTER_TOLERANCE) / (STICK_MAX - CENTER_TOLERANCE)
    else:
        return 0

#evdev takes care of polling the controller in a loop
for event in gamepad.read_loop():
    # EV_KEY: Button events, EV_ABS: Joystick events
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
        # print (absevent.event.code)
        # if absevent.event.code == ecodes.ABS_X:
        #     print("Left Stick X:", absevent.event.value)
        # elif absevent.event.code == ecodes.ABS_Y:
        #     print("Left Stick Y:", absevent.event.value)
        # elif absevent.event.code == ecodes.ABS_RX:
        #     print("Right Stick X:", absevent.event.value)
        if absevent.event.code == 2:
            
            print("Right Stick Y:", get_stick_value(absevent.event.value))

