import numpy as np  # type: ignore
import math

a1 = 0
b1 = 10
a2 = 10
b2 = 12
x1 = 0


class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        """
        Inizializza il controllore PID per un sistema con coordinate x, y.

        :param kp: Guadagno proporzionale
        :param ki: Guadagno integrale
        :param kd: Guadagno derivativo
        :param setpoint: Coordinate desiderate del sistema (setpoint) come tupla (x, y)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.previous_error = (0, 0)
        self.integral = (0, 0)

    def update(self, feedback_value, dt):
        """
        Calcola l'output del PID per ogni coordinata.

        :param feedback_value: Coordinate misurate del sistema come tupla (x, y)
        :param dt: Intervallo di tempo (in secondi) dall'ultimo aggiornamento
        :return: Output del controllore PID come tupla (x, y)
        """
        error_x = self.setpoint[0] - feedback_value[0]
        error_y = self.setpoint[1] - feedback_value[1]

        self.integral = (
            self.integral[0] + error_x * dt,
            self.integral[1] + error_y * dt,
        )

        derivative_x = (error_x - self.previous_error[0]) / dt if dt > 0 else 0
        derivative_y = (error_y - self.previous_error[1]) / dt if dt > 0 else 0

        # Calcolo dell'output
        output_x = (
            self.kp * error_x + self.ki * self.integral[0] + self.kd * derivative_x
        )
        output_y = (
            self.kp * error_y + self.ki * self.integral[1] + self.kd * derivative_y
        )

        # Aggiorna l'errore precedente
        self.previous_error = (error_x, error_y)

        return output_x, output_y

    # Esempio di utilizzo del PIDController


if __name__ == "__main__":
    import time

    def calcolo_altezze(raggio, PA, PB):
        # Coordinate di A e B
        A = PA  # Punto A (centro della piattaforma)
        B = PB  # Punto B (direzione desiderata)

        # Raggio della piattaforma
        R = raggio

        # Coordinate dei tre punti sul bordo
        P1 = (R, 0)
        P2 = (-R / 2, np.sqrt(3) * R / 2)
        P3 = (-R / 2, -np.sqrt(3) * R / 2)

        # Direzione desiderata (A -> B)
        d = (B[0] - A[0], B[1] - A[1])

        # Calcolo del piano (coefficenti a, b)
        a = d[0] / R
        b = d[1] / R

        # Calcolo delle altezze
        h1 = a * P1[0] + b * P1[1]
        h2 = a * P2[0] + b * P2[1]
        h3 = a * P3[0] + b * P3[1]

        print(f"Altezze dei tre punti:")
        print(f"h1 (P1): {h1:.2f}")
        print(f"h2 (P2): {h2:.2f}")
        print(f"h3 (P3): {h3:.2f}")

    def inverse_kinematic(L1, L2, Xt, Yt):
        theta_2 = math.acos(
            (pow(Xt, 2) + pow(Yt, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
        )
        theta_1 = math.atan2(Yt, Xt) - math.atan2(
            L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2)
        )
        return round(math.degrees(theta_1))

    # Inizializza il PID con guadagni specifici
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05, setpoint=(0, 0))

    # Simula un sistema con valori iniziali per x e y
    current_value = (5, 3)

    for i in range(5):
        # Simula l'aggiornamento del sistema ogni 0.1 secondi
        dt = 1
        # time.sleep(dt)

        # Calcola l'output del PID
        control_signal = pid.update(current_value, dt)
        print(
            "Valore precedente "
            + str(round(current_value[0], 2))
            + ", "
            + str(round(current_value[1], 2))
        )

        current_value1 = (
            current_value[0] + control_signal[0] * dt,
            current_value[1] + control_signal[1] * dt,
        )

        calcolo_altezze(12, current_value, current_value1)

        # Applica l'output del PID al sistema (simulazione)
        current_value = (
            current_value[0] + control_signal[0] * dt,
            current_value[1] + control_signal[1] * dt,
        )

        print(
            f"Tempo: {i * dt:.1f}s, Valore attuale: ({current_value[0]:.2f}, {current_value[1]:.2f}), \
                   Segnale di controllo: ({control_signal[0]:.2f}, {control_signal[1]:.2f})"
        )
        # time.sleep(dt)


def calcolo_altezze(raggio, PA, PB):
    # Coordinate di A e B
    A = PA  # Punto A (centro della piattaforma)
    B = PB  # Punto B (direzione desiderata)

    # Raggio della piattaforma
    R = raggio

    # Coordinate dei tre punti sul bordo
    P1 = (R, 0)
    P2 = (-R / 2, np.sqrt(3) * R / 2)
    P3 = (-R / 2, -np.sqrt(3) * R / 2)

    # Direzione desiderata (A -> B)
    d = (B[0] - A[0], B[1] - A[1])

    # Calcolo del piano (coefficenti a, b)
    a = d[0] / R
    b = d[1] / R

    # Calcolo delle altezze
    h1 = a * P1[0] + b * P1[1]
    h2 = a * P2[0] + b * P2[1]
    h3 = a * P3[0] + b * P3[1]

    print(f"Altezze dei tre punti:")
    print(f"h1 (P1): {h1:.2f}")
    print(f"h2 (P2): {h2:.2f}")
    print(f"h3 (P3): {h3:.2f}")


def phase_calculation1(x, y):
    phase = 0
    if x == 0 and y > 0:
        phase = math.pi / 2
    elif x == 0 and y < 0:
        phase = -(math.pi / 2)
    elif x > 0:
        phase = math.atan(y / x)
    elif x < 0 and y >= 0:
        phase = math.atan(y / x) + math.pi
    elif x < 0 and y < 0:
        phase = math.atan(y / x) - math.pi
    elif x == 0 and y == 0:
        phase = 0
    phase = round(math.degrees(phase))
    if phase < 0:
        phase = phase + 360
    return phase


def distance_calculation(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return round(distance, 1)


def height_calculation():
    l1 = linear_relation(0, 12, 8, 12, distance_1, False)
    l2 = linear_relation(0, 12, 8, 12, distance_2, False)
    l3 = linear_relation(0, 12, 8, 12, distance_3, False)
    return l1, l2, l3


def check_zone(phase):
    zone = ""

    if phase >= 330 or phase < 90:
        zone = "A"
        l1 = linear_relation(0, 12, 8, 12, distance_1, False)
        l2 = linear_relation(0, 12, 8, 12, distance_2, False)
        l3 = linear_relation(0, 12, 8, 12, distance_3, False)

    elif phase >= 90 and phase < 210:
        zone = "B"
        l1 = linear_relation(6, 12, 8, 10, distance_1, False)
        l2 = linear_relation(0, 10.5, 10, 12, distance_2, False)
        l3 = linear_relation(0, 10.5, 10, 12, distance_3, False)

    elif phase >= 210 and phase < 330:
        zone = "C"
        l1 = linear_relation(0, 6, 10, 12, distance_1, False)
        l2 = linear_relation(6, 12, 8, 10, distance_2, False)
        l3 = linear_relation(0, 6, 10, 12, distance_3, False)
    print("Zona " + zone)
    return l1, l2, l3


def linear_relation(a1, b1, a2, b2, x1, third):
    result = 0
    x2 = a2 + (x1 - a1) * (b2 - a2) / (b1 - a1)
    x2 = round(x2, 1)
    if third == True:
        result = x2
    else:
        result = b2 - x2 + a2
    return result


def inverse_kinematic(L1, L2, Xt, Yt):
    theta_2 = math.acos(
        (pow(Xt, 2) + pow(Yt, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
    )
    theta_1 = math.atan2(Yt, Xt) - math.atan2(
        L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2)
    )
    return round(math.degrees(theta_1))


# while True:
#    print("Inserisci 1")
#    x = int(input())
#   print("Inserisci 2")
#  y = int(input())

# posizione = (x, y)
# pid.update(posizione[0], posizione[1])

# t = phase_calculation1(posizione[0], posizione[1])
# print(t)
# z = check_zone(t)
# print(z)

# distance_1 = distance_calculation(5.2, -3, posizione[0], posizione[1])
# distance_2 = distance_calculation(0, 6, posizione[0], posizione[1])
# distance_3 = distance_calculation(-5.2, -3, posizione[0], posizione[1])
# print("Distanza 1 = "+str(distance_1))
# print("Distanza 2 = "+str(distance_2))
# print("Distanza 3 = "+str(distance_3))

# l1, l2 , l3 = height_calculation()

# print("Altezza 1 = "+str(l1))
# print("Altezza 2 = "+str(l2))
# print("Altezza 3 = "+str(l3))

# theta_1 = inverse_kinematic(6, 8, 0, l1)
# theta_2 = inverse_kinematic(6, 8, 0, l2)
# theta_3 = inverse_kinematic(6, 8, 0, l3)

# print("Servo 1 = "+str(theta_1))
# print("Servo 2 = "+str(theta_2))
# print("Servo 3 = "+str(theta_3))
