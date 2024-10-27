from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
while True:
    motor.write_goal_velocity(10)
    time.sleep(0.5)
    motor.write_goal_velocity(30)
    time.sleep(0.5)