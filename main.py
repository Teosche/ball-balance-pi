from api import app
from camera import Camera
from pid import PIDController
from balancer import ServoController, vision
import threading
import pigpio

if __name__ == "__main__":
    threading.Thread(
        target=app.run, kwargs={"host": "0.0.0.0", "port": 5000, "debug": False}
    ).start()

    pi = pigpio.pi()
    camera = Camera()
    pid = PIDController(kp=0.05, ki=0.0025, kd=0.025, setpoint=(0, 0))
    servo = ServoController(pi)

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=vision, args=(stop_event, camera, pid, servo)
    )
    vision_thread.start()
