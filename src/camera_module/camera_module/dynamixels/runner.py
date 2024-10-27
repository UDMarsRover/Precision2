from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
while True:
    motor.write_goal_position(2048)
    time.sleep(1.5)
    motor.write_goal_position(0)
    time.sleep(1.5)