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

    The camera provides raw coordinates for the detected circle. These are first transformed:
      - Dividing by 2,
      - Mapped using linear_relation (from range [15,120] to [-6,6]),
      - Rotated by -10°.

    After transformation, the ball's position is assumed to be in a coordinate system where
    (0, 0) is the center. The PID controller is used with setpoint (0, 0) so that any deviation
    generates a control signal. This signal is mapped to target servo commands using additional
    transformations (calcolo_altezze and inverse_kinematic).

    If no ball is detected, the servo is reset to the default setup position.

    Args:
        stop_event (threading.Event): Event to stop the loop.
        camera (Camera): The camera object.
        pid (PID): PID controller with setpoint (0, 0).
        servo (Servo): Servo controller.
    """
    dt = 0.05  # time step in seconds

    while not stop_event.is_set():
        # Capture a frame and detect circle(s)
        frame = camera.capture_frame()
        camera.detect_circle(frame)

        if camera.circle is not None:
            # Assume we take the first detected circle
            circles = np.round(camera.circle[0, :]).astype("int")
            x_raw, y_raw, r = circles[0]
            # Draw circle on frame (for debugging/overlay)
            camera.print_circle(frame, x_raw, y_raw, r)

            # Transform raw coordinates:
            pos_x = round(x_raw / 2)
            pos_y = round(y_raw / 2)

            # Applica la mappatura lineare per adattare il range
            pos_x = linear_relation(15, 120, -6, 6, pos_x, False)
            pos_y = linear_relation(15, 120, -6, 6, pos_y, False)
            # Calcola la distanza e ruota le coordinate di -10°
            raggio = math.sqrt(pos_x**2 + pos_y**2)
            angolo_attuale = math.atan2(pos_y, pos_x)
            angolo_totale = angolo_attuale + math.radians(-10)
            pos_x = raggio * math.cos(angolo_totale)
            pos_y = raggio * math.sin(angolo_totale)

            offset_x, offset_y = 13.8, 3.2
            pos_x = pos_x + offset_x
            pos_y = pos_y + offset_y

            # Qui il feedback per il PID è la posizione trasformata;
            # Se la pallina è al centro, si assume che (pos_x, pos_y) sia (0,0).
            control_signal = pid.update((pos_x, pos_y), dt)
            print(
                f"Transformed Ball: ({round(pos_x,1)}, {round(pos_y,1)}), PID: ({control_signal[0]:.2f}, {control_signal[1]:.2f})"
            )

            # Mappa il segnale PID a una direzione di correzione.
            target_x = linear_relation(-1, 1, -1, 1, control_signal[0], False)
            target_y = linear_relation(-1, 1, -1, 1, control_signal[1], False)

            # Calcola altezze in base alla direzione target.
            h1, h2, h3 = calcolo_altezze(6, [0, 0], [target_x, target_y])

            # Calcola gli angoli dei servo tramite inverse kinematics.
            theta_1 = 90 - inverse_kinematic(6.5, 9, 0, h1)
            theta_2 = 90 - inverse_kinematic(6.5, 9, 0, h2)
            theta_3 = 90 - inverse_kinematic(6.5, 9, 0, h3)

            # Limita gli angoli a un range operativo.
            min_angle, max_angle = 5, 35
            theta_1 = max(min_angle, min(theta_1, max_angle))
            theta_2 = max(min_angle, min(theta_2, max_angle))
            theta_3 = max(min_angle, min(theta_3, max_angle))

            print(theta_1)
            print(theta_2)
            print(theta_3)

            # Per un sistema a delta, potresti dover applicare una logica simmetrica.
            # Qui, per semplicità, usiamo theta_1 e theta_2 per due servo e ripetiamo theta_1 per il terzo.
            servo.move_servos((theta_1, theta_2, theta_3))
        else:
            print("No ball detected. Resetting servo to default position.")
            servo.reset_servo()

        time.sleep(dt)
