import threading
from flask import Response
import pigpio

from api import app
from camera import Camera
from pid import PID
from servo import Servo
from balancer import balance_ball


def generate_frames():
    """
    Generate captured frame.
    """
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


if __name__ == "__main__":
    pi = pigpio.pi()
    camera = Camera()

    threading.Thread(
        target=app.run,
        kwargs={"host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False},
    ).start()

    @app.route("/")
    def index():
        """
        Streaming.
        """
        return Response(
            generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    pid = PID(kp=0.021, ki=0.001, kd=0.01, setpoint=(0, 0))
    servo = Servo(pi)

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=balance_ball, args=(stop_event, camera, pid, servo)
    )
    vision_thread.start()
