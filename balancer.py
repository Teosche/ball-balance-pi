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
    Uses the camera feedback to keep the ball centered on the platform.
    If no ball is detected, resets the servo to the default (setup) position.

    Args:
        stop_event (threading.Event): Event to stop the loop.
        camera (Camera): The camera instance.
        pid (PID): The PID controller.
        servo (Servo): The servo controller.
    """
    dt = 0.05  # time step in seconds
    # Define the setpoint as the center of the image.
    # Adjust these values based on your camera's resolution.
    image_center = (320, 240)

    while not stop_event.is_set():
        # Capture a frame and detect the ball.
        frame = camera.capture_frame()
        camera.detect_circle(frame)

        # Se viene rilevata una pallina...
        if camera.circle is not None:
            pos = camera.get_position_information(frame)
            if pos is not None:
                ball_x, ball_y = pos
                # Calcola l'errore come differenza dal centro dell'immagine.
                error_x = image_center[0] - ball_x
                error_y = image_center[1] - ball_y
                # Passa l'errore al PID (qui il feedback è l'errore, oppure puoi impostare il setpoint direttamente)
                control_signal = pid.update((ball_x, ball_y), dt)

                # Mappa il segnale di controllo agli angoli dei servo.
                # NOTA: questa parte dipende dalla cinematica del tuo sistema.
                # Qui ipotizziamo che il segnale PID venga mappato linearmente a un range di angoli.
                min_angle, max_angle = 15, 55
                # Per esempio, si può usare linear_relation per ottenere valori coerenti.
                theta_x = linear_relation(
                    -abs(image_center[0]),
                    abs(image_center[0]),
                    min_angle,
                    max_angle,
                    control_signal[0],
                    False,
                )
                theta_y = linear_relation(
                    -abs(image_center[1]),
                    abs(image_center[1]),
                    min_angle,
                    max_angle,
                    control_signal[1],
                    False,
                )
                # Ipotizziamo una mappatura identica per tutti i servo, oppure potresti avere un inverse kinematics specifico.
                servo.move_servos((theta_x, theta_y, theta_x))
                print(
                    f"Ball: ({ball_x}, {ball_y}), Error: ({error_x}, {error_y}), PID: ({control_signal[0]:.2f}, {control_signal[1]:.2f})"
                )
            else:
                print("Ball detected but invalid coordinates.")
        else:
            print("No ball detected. Resetting servo.")
            # Se non viene rilevata la pallina, resetta i servo alla posizione di setup.
            servo.reset_servo()

        time.sleep(dt)
