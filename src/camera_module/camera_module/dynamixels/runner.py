from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
motor.set_velocity_mode()
print("SETTING VELOCITY MODE")
# motor.write_goal_velocity()
while True:
    motor.write_goal_velocity(200)
    time.sleep(2)
    motor.write_goal_velocity(40)
    time.sleep(2)
    motor.write_goal_velocity(-100)
    time.sleep(2)