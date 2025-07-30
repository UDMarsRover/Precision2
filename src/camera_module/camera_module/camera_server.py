from picamera2 import Picamera2
import cv2
from flask import Flask, Response, request
import threading
import time
import numpy as np

app = Flask(__name__)

# Define supported output resolutions
OUTPUT_RESOLUTIONS = {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "4k": (3840, 2160) # Actual 4K (UHD)
}

# Define supported zoom levels (multipliers)
# A zoom level of 1 means no zoom (full sensor view, then scaled to output_resolution)
# A zoom level of 2 means 2x zoom (crop to 1/2 width, 1/2 height, then scaled)
ZOOM_LEVELS = [1.0, 1.5, 2.0, 3.0, 4.0] # Example zoom levels

# Sensor resolutions for Pi Camera Module 3 (approximate)
# Use a resolution that gives enough pixels for zooming effectively
# For Camera Module 3, it's often 4608x2592 (full resolution) or similar high resolutions
SENSOR_RESOLUTIONS = {
    # Full resolution of Pi Camera Module 3
    # Choose the highest resolution that the sensor can actually provide for cropping
    "full": (4608, 2592),
    # You might also use a specific 'video' mode resolution if 'full' is too slow
    # e.g., (2304, 1296) for a 16:9 aspect ratio often used in video modes
}
SENSOR_CAPTURE_RESOLUTION = SENSOR_RESOLUTIONS["full"] # Always capture at full sensor resolution

# Default output settings for each camera
DEFAULT_OUTPUT_SETTINGS = {
    "resolution": "720p",
    "zoom": 1.0
}

# Global variables to store the latest frame from each camera
# Also store the *desired* output resolution and zoom for each camera
# Using threading.Lock to ensure thread-safe access to frames and settings
latest_camera_data = {
    0: {
        "frame": None,
        "lock": threading.Lock(),
        "current_output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "current_zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "sensor_resolution_set": SENSOR_CAPTURE_RESOLUTION # Actual resolution camera is configured to
    },
    1: {
        "frame": None,
        "lock": threading.Lock(),
        "current_output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "current_zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "sensor_resolution_set": SENSOR_CAPTURE_RESOLUTION
    }
}

# Events to signal a re-capture based on new output settings
settings_change_event = {
    0: threading.Event(),
    1: threading.Event()
}

# Picamera2 instances (initialized globally, will be created in threads)
picam2_instances = {
    0: None,
    1: None
}

# Function to initialize/reconfigure a camera to its base sensor resolution
def configure_camera_for_sensor_capture(camera_id, sensor_res_tuple):
    # If the instance already exists and is started, stop it
    if picam2_instances[camera_id] is not None:
        try:
            if picam2_instances[camera_id].started:
                picam2_instances[camera_id].stop()
                print(f"Camera {camera_id}: Stopped for sensor reconfiguration.")
                time.sleep(0.1) # Give a moment for the camera to settle
            else:
                print(f"Camera {camera_id}: Not started, proceeding to configure sensor.")
        except Exception as e:
            print(f"Error trying to stop camera {camera_id}: {e}")

    try:
        # Create a new Picamera2 instance if it doesn't exist or if stopping failed
        if picam2_instances[camera_id] is None or not isinstance(picam2_instances[camera_id], Picamera2):
            picam2_instances[camera_id] = Picamera2(camera_id)

        config = picam2_instances[camera_id].create_preview_configuration(
            main={"format": 'XRGB8888', "size": sensor_res_tuple}
        )
        picam2_instances[camera_id].configure(config)
        picam2_instances[camera_id].start()
        print(f"Camera {camera_id}: Sensor configured to {sensor_res_tuple[0]}x{sensor_res_tuple[1]}")
        time.sleep(1) # Warm-up time for the camera
    except Exception as e:
        print(f"Error configuring/starting camera {camera_id} sensor to {sensor_res_tuple}: {e}")
        picam2_instances[camera_id] = None # Mark as failed or unavailable
        return False
    return True

# Function to capture frames, apply zoom/resize, and update global latest_frame
def capture_and_process_frames(camera_id):
    # Initial sensor configuration
    sensor_res_to_use = latest_camera_data[camera_id]["sensor_resolution_set"]
    print(f"Camera {camera_id} capture thread started with sensor resolution: {sensor_res_to_use}")

    if not configure_camera_for_sensor_capture(camera_id, sensor_res_to_use):
        print(f"Initial sensor configuration failed for camera {camera_id}. Exiting thread.")
        return # Exit thread if initial setup fails

    try:
        while True:
            # Check for settings change signal (not sensor resolution, but output res/zoom)
            if settings_change_event[camera_id].is_set():
                settings_change_event[camera_id].clear() # Reset the event
                # Reconfigure the sensor if necessary (though we aim to keep it constant)
                # If you allowed changing sensor_capture_resolution, you'd reconfigure here.
                print(f"Camera {camera_id}: Output settings change requested. Sensor capture resolution remains {SENSOR_CAPTURE_RESOLUTION}")

            # Ensure camera is started before capturing
            if picam2_instances[camera_id] is None or not picam2_instances[camera_id].started:
                print(f"Camera {camera_id} not started. Attempting to restart sensor...")
                if not configure_camera_for_sensor_capture(camera_id, SENSOR_CAPTURE_RESOLUTION):
                    time.sleep(2) # Prevent busy loop if configuration repeatedly fails
                    continue # Skip frame capture for this iteration

            # Capture the full sensor frame
            full_frame = picam2_instances[camera_id].capture_array()
            sensor_height, sensor_width, _ = full_frame.shape

            # Get current desired output settings
            with latest_camera_data[camera_id]["lock"]:
                output_res_str = latest_camera_data[camera_id]["current_output_res_str"]
                zoom_level = latest_camera_data[camera_id]["current_zoom_level"]
            
            output_width, output_height = OUTPUT_RESOLUTIONS[output_res_str]

            # --- Apply Zoom (Cropping) ---
            # Calculate the dimensions of the cropped area based on zoom level
            # The cropped area will be 1/zoom_level of the sensor dimensions
            cropped_width = int(sensor_width / zoom_level)
            cropped_height = int(sensor_height / zoom_level)

            # Calculate crop start coordinates to center the crop
            start_x = (sensor_width - cropped_width) // 2
            start_y = (sensor_height - cropped_height) // 2

            # Perform the crop
            cropped_frame = full_frame[start_y : start_y + cropped_height,
                                       start_x : start_x + cropped_width]

            # --- Resize to desired output resolution ---
            processed_frame = cv2.resize(cropped_frame, (output_width, output_height), interpolation=cv2.INTER_AREA)

            # Optional: Add text overlay (camera ID, zoom, output resolution)
            if camera_id == 0:
                camera_id_str = "IR Camera"
            else:
                camera_id_str = "Zoom Camera"
            overlay_text = f"{camera_id_str} - Zoom: {zoom_level}x - Output: {output_width}x{output_height}"
            cv2.putText(processed_frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 2, cv2.LINE_AA)

            # Store the processed frame
            with latest_camera_data[camera_id]["lock"]:
                latest_camera_data[camera_id]["frame"] = processed_frame
            time.sleep(0.01) # Small delay to prevent busy-waiting
    except Exception as e:
        print(f"Camera {camera_id} capture/processing error: {e}")
    finally:
        if picam2_instances[camera_id] is not None:
            if picam2_instances[camera_id].started:
                picam2_instances[camera_id].stop()
                print(f"Camera {camera_id}: Stopped due to thread exit.")
            picam2_instances[camera_id] = None # Clear instance

# Start separate threads for each camera
camera_threads = []
for i in range(2): # For 2 cameras (0 and 1)
    thread = threading.Thread(target=capture_and_process_frames, args=(i,))
    thread.daemon = True # Allow main program to exit even if threads are running
    camera_threads.append(thread)
    thread.start()

# Generator function for streaming frames to Flask
def generate_frames(camera_id):
    while True:
        with latest_camera_data[camera_id]["lock"]:
            frame = latest_camera_data[camera_id]["frame"]

        if frame is not None:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpeg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.03) # Adjust for desired frame rate

# Flask route for dynamic video feeds
# Example URL: http://<PI_IP>:5000/stream/0?resolution=720p&zoom=2.0
@app.route('/stream/<int:camera_id>')
def stream_feed(camera_id):
    if camera_id not in [0, 1]:
        return "Invalid camera ID. Use 0 or 1.", 400

    requested_resolution = request.args.get('resolution', DEFAULT_OUTPUT_SETTINGS["resolution"]).lower()
    requested_zoom_str = request.args.get('zoom', str(DEFAULT_OUTPUT_SETTINGS["zoom"]))

    # Validate resolution
    if requested_resolution not in OUTPUT_RESOLUTIONS:
        return f"Invalid resolution. Choose from: {', '.join(OUTPUT_RESOLUTIONS.keys())}", 400
    
    # Validate zoom
    try:
        requested_zoom = float(requested_zoom_str)
        if requested_zoom not in ZOOM_LEVELS:
            return f"Invalid zoom level. Choose from: {', '.join(map(str, ZOOM_LEVELS))}", 400
    except ValueError:
        return f"Invalid zoom value: '{requested_zoom_str}'. Must be a number.", 400

    # Check if settings need to be updated for this camera
    settings_changed = False
    with latest_camera_data[camera_id]["lock"]:
        if (latest_camera_data[camera_id]["current_output_res_str"] != requested_resolution or
            latest_camera_data[camera_id]["current_zoom_level"] != requested_zoom):
            
            latest_camera_data[camera_id]["current_output_res_str"] = requested_resolution
            latest_camera_data[camera_id]["current_zoom_level"] = requested_zoom
            settings_changed = True
            print(f"Camera {camera_id}: Settings updated to Resolution={requested_resolution}, Zoom={requested_zoom}x")
        else:
            print(f"Camera {camera_id}: No settings change requested. Streaming with existing settings.")

    # Signal the camera thread to apply new settings if they changed
    if settings_changed:
        settings_change_event[camera_id].set()

    return Response(generate_frames(camera_id), mimetype='multipart/x-mixed-replace; boundary=frame')

# Basic root route for instructions (optional, but helpful)
@app.route('/')
def root():
    return f"""
    <html>
    <head><title>Pi Camera Stream API</title></head>
    <body>
        <h1>Pi Camera Stream API</h1>
        <p>Access camera feeds directly:</p>
        <ul>
            <li>Camera 0: <code>/stream/0?resolution=720p&zoom=1.0</code></li>
            <li>Camera 1: <code>/stream/1?resolution=1080p&zoom=2.0</code></li>
        </ul>
        <p>Available resolutions: {', '.join(OUTPUT_RESOLUTIONS.keys())}</p>
        <p>Available zoom levels: {', '.join(map(str, ZOOM_LEVELS))}</p>
        <p>Example: <a href="/stream/0?resolution=480p&zoom=1.5">/stream/0?resolution=480p&zoom=1.5</a></p>
        <p>Example: <a href="/stream/1?resolution=1080p&zoom=3.0">/stream/1?resolution=1080p&zoom=3.0</a></p>
    </body>
    </html>
    """

if __name__ == '__main__':
    try:
        print("Starting Flask application...")
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("Stopping application (KeyboardInterrupt)...")
    except Exception as e:
        print(f"An unexpected error occurred during application startup: {e}")
    finally:
        print("Attempting to stop all cameras...")
        for cam_id in picam2_instances:
            if picam2_instances[cam_id] is not None:
                try:
                    if picam2_instances[cam_id].started:
                        picam2_instances[cam_id].stop()
                        print(f"Camera {cam_id}: Successfully stopped.")
                    else:
                        print(f"Camera {cam_id}: Not started, no need to stop.")
                except Exception as e:
                    print(f"Error stopping camera {cam_id}: {e}")
                picam2_instances[cam_id] = None # Clear the instance
        print("All cameras stopped. Application exiting.")