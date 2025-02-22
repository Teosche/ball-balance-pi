import threading
import pigpio

from api import app, init_camera
from camera import Camera
from pid import PID
from servo import Servo
from balancer import balance_ball

if __name__ == "__main__":
    pi = pigpio.pi()
    camera = Camera()
    init_camera(camera)
    pid = PID(kp=0.05, ki=0.0025, kd=0.025, setpoint=(0, 0))

    servo = Servo(pi)

    stop_event = threading.Event()
    vision_thread = threading.Thread(
        target=balance_ball, args=(stop_event, camera, pid, servo)
    )
    vision_thread.start()

    # threading.Thread(
    #     target=app.run,
    #     kwargs={"host": "0.0.0.0", "port": 5000, "debug": False},
    # ).start()
