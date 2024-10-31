from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
motor.set_velocity_mode()
print("SETTING VELOCITY MODE")
# motor.write_goal_velocity()
while True:
    motor.write_goal_velocity(200)
    while motor.read_present_position() < 3000:
        print(motor.read_present_position())
        pass
    motor.write_goal_velocity(-100)
    while motor.read_present_position() > 100:
        print(motor.read_present_position())
        pass