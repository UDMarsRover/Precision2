from flask import Flask, Response
import cv2
from picamera2 import Picamera2
import time

app = Flask(__name__)

# A global Picamera2 object for the Flask application to use
# This is a key change from the cv2.VideoCapture model, which created a new object
# for each request. Picamera2 is designed to be a singleton.
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (854, 480)}))
picam2.start()

def generate_frames():
    while True:
        # Capture a frame as a NumPy array directly from picamera2
        # No need for a separate class to manage the camera
        frame = picam2.capture_array()
        
        # NOTE: picamera2 provides frames in BGR format by default, which is
        # what OpenCV expects. If you need to do a rotation, do it here
        # using cv2.rotate.
        # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        # Encode the frame as a JPEG image
        ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        
        if not ret:
            break

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=3000, debug=False)
    finally:
        # It's good practice to stop the camera when the application exits.
        picam2.stop()