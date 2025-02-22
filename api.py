from flask import Flask, Response

app = Flask(__name__)

camera = None


def init_camera(cam):
    """
    Initialize the global camera instance.

    Args:
        cam: An instance of the Camera class.
    """
    global camera
    camera = cam


def generate_frames():
    """
    Generate captured frames using the global camera instance.
    """
    global camera
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/")
def index():
    """
    Streaming endpoint.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )
