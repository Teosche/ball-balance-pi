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
    Calculate A, B coordinates, where A is the center of the board and B is the target direction.

    Args:
        radius: The radius of the platform.
        PA: Coordinates of point A (center of the platform).
        PB: Coordinates of point B (target direction).

    Returns:
        A tuple (h1, h2, h3) with the calculated heights at three points on the edge.
    """
    A = PA  # Center of the platform
    B = PB  # Desired target direction
    offset = 14

    R = radius

    # Coordinates of the three points on the platform edge
    P1 = (-R, 0)
    P3 = (R / 2, np.sqrt(3) * -R / 2)
    P2 = (R / 2, -np.sqrt(3) * -R / 2)

    # Desired direction (from A to B)
    d = (B[0] - A[0], B[1] - A[1])

    # Compute coefficients for the plane equation
    a = d[0] / R
    b = d[1] / R

    # Calculate heights (with an offset of 14)
    h1 = a * P1[0] + b * P1[1] + offset
    h2 = a * P2[0] + b * P2[1] + offset
    h3 = a * P3[0] + b * P3[1] + offset

    # Saturate heights at 16
    if h1 >= 16:
        h1 = 16
    if h2 >= 16:
        h2 = 16
    if h3 >= 16:
        h3 = 16

    return h1, h2, h3


def inverse_kinematic(L1, L2, Xt, Yt):
    """
    Calculate the inverse kinematic function for a 2-link manipulator.

    Args:
        L1: Length of the first link.
        L2: Length of the second link.
        Xt: Target x-coordinate.
        Yt: Target y-coordinate.

    Returns:
        The computed angle (in degrees) for the first joint.
    """
    v = (pow(Xt, 2) + pow(Yt, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
    if v > 1:
        v = 1
    if v < -1:
        v = -1
    theta_2 = math.acos(v)
    theta_1 = math.atan2(Yt, Xt) - math.atan2(
        L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2)
    )
    return round(math.degrees(theta_1))


def balance_ball(stop_event, camera: Camera, pid: PID, servo: Servo):
    dt = 0.035
    previous_theta_1 = 30
    previous_theta_2 = 30
    previous_theta_3 = 30

    while not stop_event.is_set():
        frame = camera.capture_frame()
        camera.detect_circle(frame)

        if camera.circle is not None:
            circles = np.round(camera.circle[0, :]).astype("int")
            x_raw, y_raw, r = circles[0]

            pos_x = round(x_raw / 2)
            pos_y = round(y_raw / 2)

            pos_x = linear_relation(15, 120, -6, 6, pos_x, False)
            pos_y = linear_relation(15, 120, -6, 6, pos_y, False)

            raggio = math.sqrt(pos_x**2 + pos_y**2)
            angolo_attuale = math.atan2(pos_y, pos_x)
            angolo_totale = angolo_attuale + math.radians(-10)
            pos_x = raggio * math.cos(angolo_totale)
            pos_y = raggio * math.sin(angolo_totale)

            print("POS X before offset", pos_x)
            print("POS Y vefore offset", pos_y)

            offset_x, offset_y = 13.2, 3.2
            pos_x = round(pos_x + offset_x, 1)
            pos_y = round(pos_y + offset_y, 1)

            print("POS X", pos_x)
            print("POS Y", pos_y)

            control_signal = pid.update((pos_x, pos_y), dt)
            print(f"PID Output: ({control_signal[0]:.2f}, {control_signal[1]:.2f})")

            target_x = linear_relation(-1, 1, -1, 1, control_signal[0], False)
            target_y = linear_relation(-1, 1, -1, 1, control_signal[1], False)

            print("TARGET X", target_x)
            print("TARGET Y", target_y)

            h1, h2, h3 = calcolo_altezze(6, [0, 0], [target_x, target_y])

            theta_1 = 90 - inverse_kinematic(6.5, 9, 0, h1)
            theta_2 = 90 - inverse_kinematic(6.5, 9, 0, h2)
            theta_3 = 90 - inverse_kinematic(6.5, 9, 0, h3)

            max_step = 5

            theta_1 = max(
                previous_theta_1 - max_step, min(previous_theta_1 + max_step, theta_1)
            )
            theta_2 = max(
                previous_theta_2 - max_step, min(previous_theta_2 + max_step, theta_2)
            )
            theta_3 = max(
                previous_theta_3 - max_step, min(previous_theta_3 + max_step, theta_3)
            )

            previous_theta_1 = theta_1
            previous_theta_2 = theta_2
            previous_theta_3 = theta_3

            servo.move_servos((theta_1, theta_2, theta_3))
        else:
            print("No ball detected. Resetting servo to default position.")
            servo.reset_servo()

        time.sleep(dt)
