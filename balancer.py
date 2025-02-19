import math
import time
import numpy as np
import pigpio
import collections


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


# Creiamo una media mobile per rendere i movimenti dei servo più fluidi
servo_history = collections.deque(maxlen=5)  # Salviamo gli ultimi 5 angoli dei servo


def smooth_servo_movement(theta_1, theta_2, theta_3):
    """
    Applica una media mobile per rendere più fluidi i movimenti dei servo.
    """
    servo_history.append((theta_1, theta_2, theta_3))
    avg_theta_1 = sum(t[0] for t in servo_history) / len(servo_history)
    avg_theta_2 = sum(t[1] for t in servo_history) / len(servo_history)
    avg_theta_3 = sum(t[2] for t in servo_history) / len(servo_history)
    return avg_theta_1, avg_theta_2, avg_theta_3


def balance_ball(stop_event, camera: Camera, pid: PID, servo: Servo):
    dt = 0.06  # Tempo di risposta aumentato

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

            offset_x, offset_y = 13.8, 3.2
            pos_x = pos_x + offset_x
            pos_y = pos_y + offset_y

            # Applichiamo il PID con i nuovi parametri
            control_signal = pid.update((pos_x, pos_y), dt)
            print(f"PID Output: ({control_signal[0]:.2f}, {control_signal[1]:.2f})")

            # Mappiamo il segnale PID nei movimenti dei servo
            target_x = linear_relation(-1, 1, -1, 1, control_signal[0], False)
            target_y = linear_relation(-1, 1, -1, 1, control_signal[1], False)

            h1, h2, h3 = calcolo_altezze(6, [0, 0], [target_x, target_y])

            theta_1 = 90 - inverse_kinematic(6.5, 9, 0, h1)
            theta_2 = 90 - inverse_kinematic(6.5, 9, 0, h2)
            theta_3 = 90 - inverse_kinematic(6.5, 9, 0, h3)

            min_angle, max_angle = 20, 50
            theta_1 = max(min_angle, min(theta_1, max_angle))
            theta_2 = max(min_angle, min(theta_2, max_angle))
            theta_3 = max(min_angle, min(theta_3, max_angle))

            # Rendiamo i movimenti più fluidi applicando una media mobile
            theta_1, theta_2, theta_3 = smooth_servo_movement(theta_1, theta_2, theta_3)

            print(f"Servo Angles: θ1={theta_1:.2f}, θ2={theta_2:.2f}, θ3={theta_3:.2f}")

            servo.move_servos((theta_1, theta_2, theta_3))
        else:
            print("No ball detected. Resetting servo to default position.")
            servo.reset_servo()

        time.sleep(dt)
