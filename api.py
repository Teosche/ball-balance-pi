from flask import Flask, Response
from camera import Camera

app = Flask(__name__)

# Inizializza la fotocamera
camera = Camera()


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
    Homepage.
    """
    return "<h1>Streaming Video</h1><img src='/video_feed'>"


@app.route("/video_feed")
def video_feed():
    """
    Streaming video MJPEG.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )
