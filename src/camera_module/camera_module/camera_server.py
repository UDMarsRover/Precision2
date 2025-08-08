# optimized_server.py

from picamera2 import Picamera2
import cv2
from flask import Flask, Response, request
import threading
import time
import numpy as np

# --- Global Configuration ---
app = Flask(__name__)

# Define supported output resolutions for the stream
OUTPUT_RESOLUTIONS = {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "4k": (3840, 2160)
}

# Define supported zoom levels (multipliers)
ZOOM_LEVELS = [1.0, 1.5, 2.0, 3.0, 4.0]

# --- SENSOR RESOLUTIONS FOR SPECIFIC CAMERAS ---
# Pi Camera Module 3 Wide (Approximate full resolution)
RPI_CAM_3_WIDE_RES = (4608, 2592)
# Pi High-Quality (HQ) Camera (Approximate full resolution)
RPI_HQ_CAM_RES = (4056, 3040)

# Default output settings for each camera
DEFAULT_OUTPUT_SETTINGS = {
    "resolution": "720p",
    "zoom": 1.0
}
# Global state for each camera, including the latest frame and settings
# Initialize with a blank frame to prevent the generator from failing on startup
initial_output_res = OUTPUT_RESOLUTIONS[DEFAULT_OUTPUT_SETTINGS["resolution"]]
initial_frame = np.zeros((initial_output_res[1], initial_output_res[0], 3), dtype=np.uint8)


latest_camera_data = {
    0: {
        "frame": initial_frame,
        "lock": threading.Lock(),
        "output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "picam2": None, # Picamera2 instance
    },
    1: {
        "frame": initial_frame,
        "lock": threading.Lock(),
        "output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "picam2": None,
    }
}

# --- Camera Thread Functions ---
def capture_and_process_frames(camera_id):
    """
    Dedicated thread for each camera to capture, process, and store frames.
    This thread continuously runs and adapts to settings changes without restarting the camera.
    """
    print(f"Starting capture thread for camera {camera_id}...")
    picam2 = None
    try:
        # Determine the correct sensor resolution based on camera ID
        if camera_id == 0:
            sensor_capture_resolution = RPI_CAM_3_WIDE_RES
        elif camera_id == 1:
            sensor_capture_resolution = RPI_HQ_CAM_RES
        else:
            print(f"Unknown camera ID {camera_id}. Exiting thread.")
            return

        # Initialize and configure the camera instance once
        picam2 = Picamera2(camera_id)
        config = picam2.create_preview_configuration(
            main={"size": sensor_capture_resolution, "format": "XRGB8888"}
        )
        picam2.configure(config)
        picam2.start()

        # Store the instance globally for clean shutdown
        latest_camera_data[camera_id]["picam2"] = picam2
        
        sensor_width, sensor_height = sensor_capture_resolution

        print(f"Camera {camera_id} sensor is configured to {sensor_width}x{sensor_height}. Entering capture loop.")

        while True:
            # Capture the raw frame from the camera
            full_frame = picam2.capture_array()
            
            # Get the latest desired output settings from the global state
            with latest_camera_data[camera_id]["lock"]:
                output_res_str = latest_camera_data[camera_id]["output_res_str"]
                zoom_level = latest_camera_data[camera_id]["zoom_level"]
            
            output_width, output_height = OUTPUT_RESOLUTIONS[output_res_str]

            # --- Apply Zoom (Cropping) ---
            # Calculate the dimensions of the cropped area based on zoom level
            cropped_width = int(sensor_width / zoom_level)
            cropped_height = int(sensor_height / zoom_level)
            
            # Calculate crop start coordinates to center the crop
            start_x = (sensor_width - cropped_width) // 2
            start_y = (sensor_height - cropped_height) // 2

            # Perform the crop using numpy slicing
            cropped_frame = full_frame[start_y:start_y + cropped_height,
                                       start_x:start_x + cropped_width]

            # --- Resize to desired output resolution using cv2 ---
            processed_frame = cv2.resize(cropped_frame, (output_width, output_height), interpolation=cv2.INTER_AREA)

            # Add an overlay for information
            if camera_id == 0:
                camera_id_str = "Pi Cam 3 Wide"
            else:
                camera_id_str = "Pi HQ Camera"
            overlay_text = f"{camera_id_str} - Zoom: {zoom_level}x - Output: {output_width}x{output_height}"
            cv2.putText(processed_frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv2.LINE_AA)

            # Store the processed frame in the global state
            with latest_camera_data[camera_id]["lock"]:
                latest_camera_data[camera_id]["frame"] = processed_frame
            
            # A small delay to prevent the thread from consuming too much CPU.
            time.sleep(0.01)

    except Exception as e:
        print(f"Capture thread for camera {camera_id} encountered a fatal error: {e}")
        # Mark the instance as failed
        latest_camera_data[camera_id]["picam2"] = None
        # Add a delay before restarting to prevent a busy loop
        time.sleep(5)
    finally:
        # Clean up the camera instance when the thread exits
        if picam2 and picam2.started:
            picam2.stop()
        print(f"Camera {camera_id}: Stopped due to thread exit.")
        latest_camera_data[camera_id]["picam2"] = None

# Generator function for streaming frames to Flask
def generate_frames(camera_id):
    """
    Generator that provides JPEG frames for the Flask response.
    It fetches the latest processed frame from the capture thread.
    """
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
        
        # Adjust for a desired frame rate (e.g., 30fps)
        time.sleep(1.0 / 30.0)

# --- Flask Routes ---
@app.route('/stream/<int:camera_id>')
def stream_feed(camera_id):
    """
    Main route to serve the video stream.
    Allows for dynamic resolution and zoom control via URL parameters.
    """
    if camera_id not in latest_camera_data:
        return "Invalid camera ID. Use 0 or 1.", 400

    # If the camera thread failed to start, inform the user.
    if latest_camera_data[camera_id]["picam2"] is None:
        return f"Camera {camera_id} is not available. Please check the logs.", 503

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

    # Update the global state with the new settings
    with latest_camera_data[camera_id]["lock"]:
        if (latest_camera_data[camera_id]["output_res_str"] != requested_resolution or
            latest_camera_data[camera_id]["zoom_level"] != requested_zoom):
            
            latest_camera_data[camera_id]["output_res_str"] = requested_resolution
            latest_camera_data[camera_id]["zoom_level"] = requested_zoom
            print(f"Camera {camera_id}: Settings updated to Resolution={requested_resolution}, Zoom={requested_zoom}x")
        else:
            print(f"Camera {camera_id}: No settings change requested. Streaming with existing settings.")

    return Response(generate_frames(camera_id), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def root():
    """Basic root route for instructions."""
    res_list = ', '.join(OUTPUT_RESOLUTIONS.keys())
    zoom_list = ', '.join(map(str, ZOOM_LEVELS))
    return f"""
    <html>
    <head><title>Pi Camera Stream API</title></head>
    <body>
        <h1>Optimized Pi Camera Stream API</h1>
        <p>Access camera feeds directly. The camera streams are now processed on the CPU for compatibility.</p>
        <p>Available resolutions: {res_list}</p>
        <p>Available zoom levels: {zoom_list}</p>
        <p>Example for Camera 0 (Pi Cam 3 Wide): <a href="/stream/0?resolution=720p&zoom=1.0">/stream/0?resolution=720p&zoom=1.0</a></p>
        <p>Example for Camera 1 (Pi HQ Camera): <a href="/stream/1?resolution=1080p&zoom=2.0">/stream/1?resolution=1080p&zoom=2.0</a></p>
    </body>
    </html>
    """

def main():
    """Starts the camera threads and the Flask application."""
    # Start separate threads for each camera
    camera_threads = []
    for i in range(len(latest_camera_data)):
        thread = threading.Thread(target=capture_and_process_frames, args=(i,))
        thread.daemon = True
        camera_threads.append(thread)
        thread.start()

    try:
        print("Starting Flask application...")
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Shutting down...")
        # Graceful shutdown is handled by the daemon threads exiting when the main process stops
        for cam_id in latest_camera_data:
            if latest_camera_data[cam_id]["picam2"]:
                latest_camera_data[cam_id]["picam2"].stop()
        print("All cameras stopped. Application exiting.")

if __name__ == '__main__':
    main()
