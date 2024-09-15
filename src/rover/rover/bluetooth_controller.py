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

right_stick_x_raw = 0
right_stick_y_raw = 0
left_stick_x_raw = 0
left_stick_y_raw = 0
a_toggle_state = False

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
        if absevent.event.code == 5:
            right_stick_y_raw = absevent.event.value
        if absevent.event.code == 2:
            right_stick_x_raw = absevent.event.value
        if absevent.event.code == 1:
            left_stick_y_raw = absevent.event.value
        if absevent.event.code == 0:
            left_stick_x_raw = absevent.event.value
    if event.type == ecodes.EV_KEY:
        evevent = categorize(event)
        if evevent.event.code == ecodes.BTN_A:
            if evevent.event.value == 1:
                print("A pressed")
            a_toggle_state = not a_toggle_state
    if a_toggle_state:
        print("Right Stick: Y:\t", format(get_stick_value(right_stick_y_raw), '.2f'), "\tX:\t", format(get_stick_value(right_stick_x_raw), '.2f'))
    else:
        print("Left Stick: Y:\t", format(get_stick_value(left_stick_y_raw), '.2f'), "\tX:\t", format(get_stick_value(left_stick_x_raw), '.2f'))
