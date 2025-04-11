from flask import Flask, Response
import pigpio

from camera import Camera


app = Flask(__name__)

pi = pigpio.pi()
camera = Camera()


def generate_frames():
    """
    Generate captured frame for MJPEG streaming.
    """
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/")
def index():
    """
    MJPEG video stream endpoint.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5000)
