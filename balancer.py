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


def vision(stop_event, camera: Camera, pid: PID, servo: Servo):
    """
    Process camera feed and adjust servo motors based on PID control.

    Args:
        stop_event (threading.Event): Event to stop the loop.
        camera (Camera): Camera object.
        pid (PID): PID controller.
        servo (Servo): Servo controller.
    """
    prev_x, prev_y = None, None

    if camera.circle is not None:
        x, y = camera.get_position_information(frame)
        if x is not None and y is not None:
            # continua con il calcolo e il controllo
            if prev_x is None or prev_y is None:
                prev_x, prev_y = x, y
            speed = camera.calculate_speed(x, y, prev_x, prev_y)
            prev_x, prev_y = x, y

            control_signal = pid.update((x, y), 0.05)
            target_x = linear_relation(-1, 1, -1, 1, control_signal[0], False)
            target_y = linear_relation(-1, 1, -1, 1, control_signal[1], False)

            h1, h2, h3 = calcolo_altezze(6, [0, 0], [target_x, target_y])

            theta_1 = 90 - inverse_kinematic(6.5, 9, 0, h1)
            theta_2 = 90 - inverse_kinematic(6.5, 9, 0, h2)
            theta_3 = 90 - inverse_kinematic(6.5, 9, 0, h3)

            minA, maxA = 15, 55
            theta_1 = max(minA, min(theta_1, maxA))
            theta_2 = max(minA, min(theta_2, maxA))
            theta_3 = max(minA, min(theta_3, maxA))

            servo.move_servos((theta_1, theta_2, theta_3))

            print(f"Ball position: (X: {x}, Y: {y}), Speed: {speed}")
        else:
            print("No valid ball coordinates detected.")

            time.sleep(0.1)
