import pygame
import time
import socket

# Klipper Unix socket settings
# The default path is /tmp/klippy_uds, but it can be different depending on your setup.
KLIPPER_SOCKET = "/tmp/klippy_uds"

# G-code command parameters
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

def send_gcode_to_klipper(command):
    """Sends a G-code command to Klipper via its Unix socket."""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(KLIPPER_SOCKET)
            # Klipper's API expects JSON messages terminated by an ASCII 0x03 character.
            # For simple G-code scripts, a single-line JSON RPC message works well.
            json_rpc = f'{{"jsonrpc": "2.0", "method": "gcode.script", "params": {{"script": "{command}"}}}}'
            message = json_rpc.encode() + b'\x03'
            sock.sendall(message)
    except FileNotFoundError:
        print(f"Klipper socket not found at {KLIPPER_SOCKET}. Is Klipper running?")
    except ConnectionRefusedError:
        print("Connection to Klipper refused. Is Klipper running?")
    except Exception as e:
        print(f"An error occurred while sending G-code: {e}")

def main():
    try:
        print("Starting real-time joystick control...")
        print(f"Attempting to connect to Klipper via {KLIPPER_SOCKET}")

        last_angle = 0.0
        
        # Continuously read joystick input
        while True:
            pygame.event.pump()
            axis_val = joystick.get_axis(0)
            
            # Apply a deadzone
            if abs(axis_val) < DEADZONE:
                axis_val = 0.0

            angle = map_range(axis_val, -1, 1, ANGLE_MIN, ANGLE_MAX)
            
            # Send command only if the angle has changed significantly
            if abs(angle - last_angle) > 0.1 or axis_val == 0.0 and last_angle != 0.0:
                speed = 50 * abs(axis_val) + 10
                
                # Check if we need to send a stop command
                if axis_val == 0.0:
                    gcode = f"{MACRO_NAME} ANGLE={angle:.2f} SPEED=0"
                else:
                    gcode = f"{MACRO_NAME} ANGLE={angle:.2f} SPEED={int(speed)}"

                send_gcode_to_klipper(gcode)
                last_angle = angle

            time.sleep(0.05) # Reduced sleep time for better responsiveness

    except KeyboardInterrupt:
        print("Script stopped by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Stop the stepper and quit PyGame on exit
        print("Sending final stop command to Klipper...")
        send_gcode_to_klipper(f"{MACRO_NAME} ANGLE={last_angle:.2f} SPEED=0")
        pygame.quit()

if __name__ == "__main__":
    main()