import threading
import time
import cv2
from flask import Flask, Response
import numpy as np
import pigpio

from camera import Camera
from pid import PID
from servo import Servo
from balancer import balance_ball

app = Flask(__name__)

pi = pigpio.pi()
camera = Camera()
pid = PID(kp=0.021, ki=0.001, kd=0.01, setpoint=(0, 0))
servo = Servo(pi)


def generate_frames():
    """
    Generate JPEG-encoded frames from the camera.
    """
    while True:
        frame = camera.get_frame()
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")


@app.route("/")
def index():
    """
    Video stream route.
    """
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":

    while True:
        frame_jpeg = camera.get_frame()
        frame = cv2.imdecode(np.frombuffer(frame_jpeg, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow("Camera Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(0.03)

        stop_event = threading.Event()
        vision_thread = threading.Thread(
            target=balance_ball, args=(stop_event, camera, pid, servo)
        )
        vision_thread.daemon = True
        vision_thread.start()

    cv2.destroyAllWindows()

    # stop_event = threading.Event()
    # vision_thread = threading.Thread(
    #     target=balance_ball, args=(stop_event, camera, pid, servo)
    # )
    # vision_thread.daemon = True
    # vision_thread.start()

    app.run(debug=True, use_reloader=False, host="0.0.0.0", port=5000)
