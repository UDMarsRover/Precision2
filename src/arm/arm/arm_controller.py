import pygame
import time
from moonrakerpy import MoonrakerPrinter

# Moonraker API settings
MOONRAKER_URL = "http://192.168.0.209:7125"
MACRO_NAME = "MOVE_WRIST"
ANGLE_MIN = -20
ANGLE_MAX = 20
DEADZONE = 0.05  # A small deadzone for the joystick to prevent jitter

# PyGame joystick setup
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected.")
joystick = pygame.joystick.Joystick(0)
joystick.init()

def map_range(value, in_min, in_max, out_min, out_max):
    """Maps a value from one range to another."""
    return out_min + (float(value - in_min) / (in_max - in_min)) * (out_max - out_min)

def send_move_wrist(client, angle, speed):
    """Sends the MOVE_WRIST macro command to Klipper."""
    speed=120
    gcode = f"{MACRO_NAME} ANGLE={angle:.2f} SPEED={int(speed)}"
    client.send_gcode(gcode)

def main():
    try:
        # Correctly instantiate the Moonraker client
        client = MoonrakerPrinter(MOONRAKER_URL)
        print("Connected to Moonraker API.")

        last_angle = 0
        last_time = time.time()

        while True:
            # Handle PyGame events
            pygame.event.pump()
            axis_val = joystick.get_axis(0)
            # print(f"Joystick axis value: {axis_val:.2f}")

            # Apply a deadzone to prevent jitter
            if abs(axis_val) < DEADZONE:
                axis_val = 0.0

            angle = map_range(axis_val, -1, 1, ANGLE_MIN, ANGLE_MAX)
            now = time.time()
            dt = now - last_time

            # The original speed calculation was complex and potentially inaccurate
            # A simple multiplier gives more predictable, smoother control
            speed = 50 * abs(axis_val) + 10 # A base speed plus a joystick-based speed.

            # Send a command only if the angle has changed significantly
            if abs(angle - last_angle) > 0.1:
                send_move_wrist(client, angle, speed)
                last_angle = angle
                last_time = now

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Script stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()