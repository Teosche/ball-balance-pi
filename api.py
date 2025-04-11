from flask import Flask, Response

app = Flask(__name__)

# Global camera instance, will be set via init_camera() from main.py
camera = None


def init_camera(cam):
    """
    Initialize the global camera instance for API streaming.

    Args:
        cam: An instance of the Camera class.
    """
    global camera
    camera = cam


def generate_frames():
    """
    Generate captured frame.
    """
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/")
def index():
    """
    Streaming.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )
