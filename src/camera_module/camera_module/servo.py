from gpiozero import Servo
from time import sleep

# Define the GPIO pin connected to the servo's signal wire
# GPIO 12 is physical pin 32 on the Raspberry Pi's 40-pin header
SERVO_PIN = 12

# Initialize the servo object
# The 'min_pulse_width' and 'max_pulse_width' define the range of the servo's movement.
# Standard values are 1ms to 2ms (1000us to 2000us).
# gpiozero's Servo class maps -1 to min_pulse_width and 1 to max_pulse_width.
# You might need to adjust these values slightly for your specific servo
# to get its full range without straining it.
# For example, if your servo overshoots or doesn't reach the full extent,
# you can adjust the min_pulse_width and max_pulse_width parameters.
# servo = Servo(SERVO_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
# The default values are usually good for a start:
servo = Servo(SERVO_PIN)

print(f"Servo connected to GPIO {SERVO_PIN}. Moving back and forth every 2 seconds.")
print("Press Ctrl+C to stop the script.")

try:
    while True:
        # Move servo to one extreme (e.g., -90 degrees)
        print("Moving servo to min position (-1)...")
        servo.min()
        sleep(2)  # Wait for 2 seconds

        # Move servo to the other extreme (e.g., +90 degrees)
        print("Moving servo to max position (1)...")
        servo.max()
        sleep(2)  # Wait for 2 seconds

except KeyboardInterrupt:
    # This block is executed when Ctrl+C is pressed
    print("\nStopping servo and cleaning up GPIO...")
    servo.close() # Release the GPIO pin
    print("Script terminated.")

