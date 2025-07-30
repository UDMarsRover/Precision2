from flask import Flask, Response
import cv2 # or your camera library

app = Flask(__name__)

# Emulate a camera or use your actual camera setup
class Camera:
    def __init__(self):
        self.video = cv2.VideoCapture(0) # Use 0 for default webcam
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 854)  # <<< CHANGE THIS VALUE FOR WIDTH
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # <<< CHANGE THIS VALUE FOR HEIGHT


    def get_frame(self):
        success, frame = self.video.read()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # Rotate the frame if needed
        if not success:
            return None
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        ret, jpeg = cv2.imencode('.jpg', frame, encode_param)
        return jpeg.tobytes()

camera = Camera()

def generate_frames():
    while True:
        frame = camera.get_frame()
        if frame is None:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3000, debug=False)
