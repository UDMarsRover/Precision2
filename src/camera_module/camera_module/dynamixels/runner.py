from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
motor.set_velocity_mode()
print("SETTING VELOCITY MODE")
# motor.write_goal_velocity()
while True:
    motor.write_goal_velocity(200)
    while motor.read_present_velocity() < motor.get_max_position():
        pass
    motor.write_goal_velocity(-100)
    while motor.read_present_velocity() > motor.get_min_position():
        pass