import requests
import time

# --- Configuration ---
# This URL points to Moonraker running on the same device.
# The default port is 7125.
MOONRAKER_URL = "http://localhost:7125"

# Set the movement speed (feed rate) in mm/min.
# A lower number makes the movement slower and smoother.
MOVEMENT_SPEED_MM_PER_MIN = 1000

# Set the range and step for the smooth movement.
# This will move from START_POSITION to END_POSITION, in steps of STEP_SIZE.
START_POSITION = 0.0
END_POSITION = 200.0
STEP_SIZE = 1.0


def send_gcode_command(command):
    """
    Sends a G-code command to the Moonraker API.
    
    Args:
        command (str): The G-code command to execute.
    """
    api_endpoint = f"{MOONRAKER_URL}/printer/gcode/script"
    
    # The payload for the API request
    payload = {
        "script": command
    }

    print(f"Sending command: {command}")

    try:
        response = requests.post(api_endpoint, json=payload)
        response.raise_for_status()  # Raises an HTTPError for bad responses (4xx or 5xx)
        
        # Check if the response was successful
        if response.status_code == 200:
            print("Command sent successfully.")
        else:
            print(f"Error sending command: {response.status_code} - {response.text}")
    
    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")


def main():
    """
    Main function to execute the homing and movement sequence.
    """
    print("--- Starting Moonraker Homing and Smooth Movement Script ---")

    # 1. Home the X-axis
    print("\n--- Homing the X-axis ---")
    send_gcode_command("G28 X")
    
    # It's a good practice to wait a bit after homing before issuing new commands.
    # The homing process itself takes time.
    time.sleep(5)
    
    # 2. Move smoothly through the defined range
    print("\n--- Starting smooth movement along X-axis ---")
    
    # Iterate from the start to the end position
    current_position = START_POSITION
    while current_position <= END_POSITION:
        # Create the G-code command for the move.
        # G1 is for linear move, X is the axis, and F is the feed rate.
        command = f"G1 X{current_position} F{MOVEMENT_SPEED_MM_PER_MIN}"
        send_gcode_command(command)
        
        # Increment the position for the next move
        current_position += STEP_SIZE
        
        # A small delay to make the movement smoother and not overwhelm the API
        time.sleep(0.05)
        
    print("\n--- Movement complete. ---")


if __name__ == "__main__":
    main()

