from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
while True:
    motor.write_goal_position(8000)
    time.sleep(2)
    motor.write_goal_position(0)
    time.sleep(2)