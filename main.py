import threading
import pigpio

from camera import Camera
from pid import PID
from servo import Servo
from balancer import balance_ball

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
