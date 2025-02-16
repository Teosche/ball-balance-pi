from flask import Flask, Response

app = Flask(__name__)


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
