import threading
import pigpio

from api import app
from camera import Camera
from pid import PID
from servo import Servo
from balancer import vision


if __name__ == "__main__":
    pi = pigpio.pi()
    camera = Camera()
    pid = PID(kp=0.05, ki=0.0025, kd=0.025, setpoint=(0, 0))
    servo = Servo(pi)

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=vision, args=(stop_event, camera, pid, servo)
    )
    vision_thread.start()

    # threading.Thread(
    #     target=app.run,
    #     args=(camera),
    #     kwargs={"host": "0.0.0.0", "port": 5000, "debug": False},
    # ).start()
