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

# SENSOR RESOLUTIONS FOR SPECIFIC CAMERAS
RPI_CAM_3_WIDE_RES = (4608, 2592)
RPI_HQ_CAM_RES = (4056, 3040)

# New scale factor to reduce the input image size for efficiency.
INPUT_RESOLUTION_SCALE = 0.5

# Default output settings for each camera
DEFAULT_OUTPUT_SETTINGS = {
    "resolution": "720p",
    "zoom": 1.0
}
# Global state for each camera, including the latest frame and settings
# The initial frame is set to the rotated dimensions (height, width)
initial_output_res = OUTPUT_RESOLUTIONS[DEFAULT_OUTPUT_SETTINGS["resolution"]]
initial_frame = np.zeros((initial_output_res[1], initial_output_res[0], 3), dtype=np.uint8)

latest_camera_data = {
    0: {
        "frame": initial_frame,
        "lock": threading.Lock(),
        "output_res_str": DEFAULT_OUTPUT_SETTINGS["resolution"],
        "zoom_level": DEFAULT_OUTPUT_SETTINGS["zoom"],
        "picam2": None,
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
    """
    print(f"Starting capture thread for camera {camera_id}...")
    picam2 = None
    try:
        # Determine the correct sensor resolution based on camera ID
        if camera_id == 0:
            base_resolution = RPI_CAM_3_WIDE_RES
        elif camera_id == 1:
            base_resolution = RPI_HQ_CAM_RES
        else:
            print(f"Unknown camera ID {camera_id}. Exiting thread.")
            return

        # Apply the scaling factor to the base resolution for efficiency
        scaled_width = int(base_resolution[0] * INPUT_RESOLUTION_SCALE)
        scaled_height = int(base_resolution[1] * INPUT_RESOLUTION_SCALE)
        sensor_capture_resolution = (scaled_width, scaled_height)

        picam2 = Picamera2(camera_id)
        config = picam2.create_preview_configuration(
            main={"size": sensor_capture_resolution, "format": "XRGB8888"}
        )
        picam2.configure(config)
        picam2.start()

        latest_camera_data[camera_id]["picam2"] = picam2
        sensor_width, sensor_height = sensor_capture_resolution

        print(f"Camera {camera_id} sensor is configured to {sensor_width}x{sensor_height}. Entering capture loop.")

        while True:
            full_frame = picam2.capture_array()
            
            with latest_camera_data[camera_id]["lock"]:
                output_res_str = latest_camera_data[camera_id]["output_res_str"]
                zoom_level = latest_camera_data[camera_id]["zoom_level"]
            
            output_width, output_height = OUTPUT_RESOLUTIONS[output_res_str]

            # --- Apply Zoom (Cropping) ---
            cropped_width = int(sensor_width / zoom_level)
            cropped_height = int(sensor_height / zoom_level)
            
            start_x = (sensor_width - cropped_width) // 2
            start_y = (sensor_height - cropped_height) // 2

            cropped_frame = full_frame[start_y:start_y + cropped_height,
                                       start_x:start_x + cropped_width]

            # --- Resize and Rotate ---
            processed_frame = cv2.resize(cropped_frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
            processed_frame = cv2.rotate(processed_frame, cv2.ROTATE_90_CLOCKWISE)

            with latest_camera_data[camera_id]["lock"]:
                latest_camera_data[camera_id]["frame"] = processed_frame
            
            time.sleep(0.01)

    except Exception as e:
        print(f"Capture thread for camera {camera_id} encountered a fatal error: {e}")
        latest_camera_data[camera_id]["picam2"] = None
        time.sleep(5)
    finally:
        if picam2 and picam2.started:
            picam2.stop()
        print(f"Camera {camera_id}: Stopped due to thread exit.")
        latest_camera_data[camera_id]["picam2"] = None

# --- Streaming Generator and Flask Routes ---
def generate_frames(camera_id):
    """Generator that provides JPEG frames for the Flask response."""
    while True:
        with latest_camera_data[camera_id]["lock"]:
            frame = latest_camera_data[camera_id]["frame"]
        if frame is not None:
            ret, buffer = cv2.imencode('.jpeg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(1.0 / 30.0)

@app.route('/stream/<int:camera_id>')
def stream_feed(camera_id):
    """Main route to serve the video stream."""
    if camera_id not in latest_camera_data:
        return "Invalid camera ID. Use 0 or 1.", 400
    if latest_camera_data[camera_id]["picam2"] is None:
        return f"Camera {camera_id} is not available. Please check the logs.", 503

    requested_resolution = request.args.get('resolution', DEFAULT_OUTPUT_SETTINGS["resolution"]).lower()
    requested_zoom_str = request.args.get('zoom', str(DEFAULT_OUTPUT_SETTINGS["zoom"]))

    if requested_resolution not in OUTPUT_RESOLUTIONS:
        return f"Invalid resolution. Choose from: {', '.join(OUTPUT_RESOLUTIONS.keys())}", 400
    
    try:
        requested_zoom = float(requested_zoom_str)
        if requested_zoom not in ZOOM_LEVELS:
            return f"Invalid zoom level. Choose from: {', '.join(map(str, ZOOM_LEVELS))}", 400
    except ValueError:
        return f"Invalid zoom value: '{requested_zoom_str}'. Must be a number.", 400

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
        <h1>Cleaned-up Pi Camera Stream API</h1>
        <p>Access camera feeds directly. Streams are rotated 90 degrees.</p>
        <p>Available resolutions: {res_list}</p>
        <p>Available zoom levels: {zoom_list}</p>
        <p>Example for Camera 0: <a href="/stream/0?resolution=720p&zoom=1.0">/stream/0?resolution=720p&zoom=1.0</a></p>
        <p>Example for Camera 1: <a href="/stream/1?resolution=1080p&zoom=2.0">/stream/1?resolution=1080p&zoom=2.0</a></p>
    </body>
    </html>
    """

def main():
    """Starts the camera threads and the Flask application."""
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
        for cam_id in latest_camera_data:
            if latest_camera_data[cam_id]["picam2"]:
                latest_camera_data[cam_id]["picam2"].stop()
        print("All cameras stopped. Application exiting.")

if __name__ == '__main__':
    main()
