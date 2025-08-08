import time

import RPi.GPIO as GPIO

SERVO_PIN = 12
FREQUENCY = 50  # Hz

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)

try:
    while True:
        # Move to 0 degrees
        set_angle(0)
        time.sleep(1)
        # Move to 180 degrees
        set_angle(180)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()