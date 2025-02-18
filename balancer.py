import math
import time
import numpy as np
import pigpio

from pid import PID
from camera import Camera
from servo import Servo

SERVO_PIN = 18
SERVO_PIN1 = 23
SERVO_PIN2 = 24

pi = pigpio.pi()


def setup_servo():
    """
    Setup the servo at calibration values
    """
    pulse = 500 + (30 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    pulse1 = 500 + (30 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
    pulse2 = 500 + (30 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN2, pulse2)
    time.sleep(2)


def linear_relation(a1, b1, a2, b2, x1, third):
    result = 0
    x2 = a2 + (x1 - a1) * (b2 - a2) / (b1 - a1)
    x2 = round(x2, 1)
    if third == True:
        result = x2
    else:
        result = b2 - x2 + a2
    return result


def calcolo_altezze(radius, PA, PB):
    """
    Calculate A, B coordinates, where A is the center of the board and B is the target direction

    Args:

    radius:
    PA:
    PB:
    """
    # Coordinate di A e B
    A = PA  # Punto A (centro della piattaforma)
    B = PB  # Punto B (direzione desiderata)

    # radius
    R = radius

    # Coordinate dei tre punti sul bordo
    P1 = (-R, 0)
    P3 = (R / 2, np.sqrt(3) * -R / 2)
    P2 = (R / 2, -np.sqrt(3) * -R / 2)

    # Direzione desiderata (A -> B)
    d = (B[0] - A[0], B[1] - A[1])

    # Calcolo del piano (coefficenti a, b)
    a = d[0] / R
    b = d[1] / R

    # Calcolo delle altezze
    h1 = a * P1[0] + b * P1[1] + 14
    h2 = a * P2[0] + b * P2[1] + 14
    h3 = a * P3[0] + b * P3[1] + 14

    if h1 >= 16:
        h1 = 16
    if h2 >= 16:
        h2 = 16
    if h3 >= 16:
        h3 = 16

    return h1, h2, h3


def inverse_kinematic(L1, L2, Xt, Yt):
    """
    Calculate the inverse kinematic function
    """
    v = (pow(Xt, 2) + pow(Yt, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
    if v > 1:
        v = 1
    if v < -1:
        v = 1
    theta_2 = math.acos(v)
    theta_1 = math.atan2(Yt, Xt) - math.atan2(
        L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2)
    )
    return round(math.degrees(theta_1))


def balance_ball(stop_event, camera: Camera, pid: PID, servo: Servo):
    """
    Use camera feedback to level the plate and center the ball.

    The image center is defined as (360, 240). The PID controller is set to bring the
    error (ball position minus image center) to (0, 0). If no ball is detected,
    the servos are reset to the default setup position.

    Args:
        stop_event (threading.Event): Event to stop the loop.
        camera (Camera): The camera object.
        pid (PID): PID controller with setpoint (0, 0).
        servo (Servo): Servo controller.
    """
    dt = 0.05  # time step in seconds
    image_center = (360, 240)

    while not stop_event.is_set():
        frame = camera.capture_frame()
        camera.detect_circle(frame)

        pos = camera.get_position_information(frame)
        if pos is not None:
            ball_x, ball_y = pos
            # Compute error relative to the center: positive error means ball is to the right/bottom.
            error_x = ball_x - image_center[0]
            error_y = ball_y - image_center[1]
            print(f"Ball: ({ball_x}, {ball_y}), Error: ({error_x}, {error_y})")

            # Update PID using the error, with desired setpoint = (0, 0)
            control_signal = pid.update((error_x, error_y), dt)
            print(f"PID output: ({control_signal[0]:.2f}, {control_signal[1]:.2f})")

            # Map the PID output to servo angles.
            # Qui assumiamo una mappatura lineare: partiamo da un angolo di base (es. 30°)
            # e aggiungiamo una correzione proporzionale al segnale PID.
            # (La scala va calibrare in base al tuo sistema.)
            base_angle = 30
            scale = 0.5  # fattore di scala da regolare empiricamente
            theta_x = base_angle + control_signal[0] * scale
            theta_y = base_angle + control_signal[1] * scale

            # Per un sistema a delta potresti voler applicare una logica simmetrica;
            # qui per semplicità usiamo theta_x e theta_y per due dei servo e ripetiamo theta_x per il terzo.
            min_angle, max_angle = 15, 55
            theta_x = max(min_angle, min(theta_x, max_angle))
            theta_y = max(min_angle, min(theta_y, max_angle))

            servo.move_servos((theta_x, theta_y, theta_x))
        else:
            print("No ball detected. Resetting servo to default.")
            servo.reset_servo()

        time.sleep(dt)
