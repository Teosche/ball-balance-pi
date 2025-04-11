import threading
from flask import Flask, Response
import pigpio

from camera import Camera
from pid import PID
from servo import Servo
from balancer import balance_ball

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
    Streaming endpoint.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    pi = pigpio.pi()
    camera = Camera()
    pid = PID(kp=0.021, ki=0.001, kd=0.01, setpoint=(0, 0))
    servo = Servo(pi)

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=balance_ball, args=(stop_event, camera, pid, servo)
    )
    vision_thread.start()

    flask_thread = threading.Thread(
        target=app.run,
        kwargs={"host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False},
    )
    flask_thread.start()

    vision_thread.join()
    flask_thread.join()
