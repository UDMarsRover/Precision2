from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
motor.set_velocity_mode()
while True:
    motor.write_goal_velocity(10)
    time.sleep(2)
    motor.write_goal_velocity(0)
    time.sleep(2)