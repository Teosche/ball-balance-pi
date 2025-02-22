# import RPi.GPIO as GPIO
import pigpio
import time
import math
import threading
from random import *
from picamera2 import Picamera2
import cv2
import numpy as np

SERVO_PIN = 18
SERVO_PIN1 = 23
SERVO_PIN2 = 24

lock = True
lock1= True
print("Caricamento...")
pi = pigpio.pi()
if not pi.connected:
    print("Errore pigpio")
    exit(1)
angolo1 = 0
angolo2 = 0
angolo3 = 0


t = 0.05
dt = 0.05
posizione_x = 0
posizione_y = 0
velocita = 0
t1=30
t2=31
t3=28

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (240, 240)})

picam2.configure(camera_config)

picam2.start()


stop_event1 = threading.Event()

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

def linear_relation(a1, b1, a2, b2, x1, third):
    result = 0
    x2 = a2 + (x1 - a1) * (b2 - a2) / (b1 - a1)
    x2 = round(x2, 1)
    if third == True:
        result = x2
    else:
        result = b2 - x2 + a2
    return result
    
def calcolo_altezze(raggio, PA, PB):
    # Coordinate di A e B
    A = PA  # Punto A (centro della piattaforma)
    B = PB  # Punto B (direzione desiderata)

    # Raggio della piattaforma
    R = raggio

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

    #print(f"Altezze dei tre punti:")
    #print(f"h1 (P1): {h1:.2f}")
    #print(f"h2 (P2): {h2:.2f}")
    #print(f"h3 (P3): {h3:.2f}")
    return h1,h2,h3
    
def inverse_kinematic(L1, L2, Xt, Yt):
    v=(pow(Xt, 2) + pow(Yt, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2)
    if v>1:
        v=1
    if v<-1:
        v=1
    theta_2 = math.acos(
       v
    )
    theta_1 = math.atan2(Yt, Xt) - math.atan2(
        L2 * math.sin(theta_2), L1 + L2 * math.cos(theta_2)
    )
    return round(math.degrees(theta_1))


def vision(stop_event1):
    try:
        global posizione_x
        global posizione_y
        global t1
        global t2
        global t3
        
        pid = PIDController(kp=0.05, ki=0.0005, kd=0.019, setpoint=(0.7, 0.8)) ################################
        lock = True
        lock1 = True
        posizione_x_prec = 10
        posizione_y_prec = 120
       
        global velocita
        prevCircle = None
        dist = lambda x1, y1, x2, y2: (x1 - x2) ** 2 + (y1 - y2) ** 2
        while not stop_event1.is_set():
            # Legge un frame dal video
            frame = picam2.capture_array()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurFrame = cv2.GaussianBlur(gray, (17, 17), 0)
            circles = cv2.HoughCircles(
                blurFrame,
                cv2.HOUGH_GRADIENT,
                1.2,
                100000,
                param1=100,
                param2=30,
                minRadius=5,
                maxRadius=80,
            )

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                for x, y, r in circles:
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
                    posizione_x = round(x / 2)
                    posizione_y = round(y / 2) #########################################################
                    #print(f"Posizione: (X:{posizione_x}: Y:{posizione_y})")
                    d=-7
                    posizione_x = linear_relation(15, 120, -6, 6, posizione_x, False)
                    posizione_y = linear_relation(15, 120, -6, 6, posizione_y, False)
                    raggio = math.sqrt(posizione_x**2 + posizione_y**2)
                    angolo_attuale = math.atan2(posizione_y, posizione_x)
                    angolo_totale = angolo_attuale + math.radians(d)
                    posizione_x = raggio * math.cos(angolo_totale)
                    posizione_y = raggio * math.sin(angolo_totale)
                    if lock is True:
                        posizione_x_prec = posizione_x
                        posizione_y_prec = posizione_y
                        lock = False

                    
                    mod = math.sqrt(
                        (posizione_x - posizione_x_prec) ** 2
                        + (posizione_y - posizione_y_prec) ** 2
                    )
                    velocita = round(mod, 1)
                    posizione_x = round(posizione_x,1)
                    posizione_y= round(posizione_y,1)
                    print(f"Posizione: (X:{posizione_x}: Y:{posizione_y})")

                    posizione_x_prec = round(posizione_x,1)
                    posizione_y_prec = round(posizione_y,1)
                    current_value = [posizione_x, posizione_y]
                    
                    
                    control_signal = pid.update(current_value, dt)
                    c1 = linear_relation(-1,1,-1,1,control_signal[0],False)
                    c2 = linear_relation(-1,1,-1,1,control_signal[1],False)
                    c1 = round(c1,1)
                    c2 = round(c2,1)
                    
                    #print("C1 = "+str(c1))
                    #print("C2 = "+str(c2))
                    h1, h2, h3 = calcolo_altezze(6, [0,0], [c1,c2])
                    
                   
                    h1 = round(h1,1)
                    h2 = round(h2,1)
                    h3 = round(h3,1)
                    #print(h1)
                    #print(h2)
                    #print(h3)
                    
                    theta_1 = 90-inverse_kinematic(6.5, 9, 0, h1)
                    theta_2 = 90-inverse_kinematic(6.5, 9, 0, h2)
                    theta_3 =90-inverse_kinematic(6.5, 9, 0, h3)
                    minA = 15
                    maxA = 55 ##########################
                    if theta_1 > maxA:
                        theta_1 = maxA
                    if theta_1 < minA:
                        theta_1 = minA
                    if theta_2 > maxA:
                        theta_2 = maxA
                    if theta_2 < minA:
                        theta_2 = minA
                    if theta_3 > maxA:
                        theta_3 = maxA
                    if theta_3 < minA:
                        theta_3 = minA
                    t1 = theta_1
                    t2 = theta_2
                    t3 = theta_3
                    
                        
                    #print("Theta1= "+str(t1))
                    #print("Theta2= "+str(t2))
                    #print("Theta3 = "+str(t3))
                    #print(c1)
                    #print(c2)
                    
                    time.sleep(t)
            else:
               print("No sfera")
                
             

    # Mostra il frame in una finestra
    # cv2.imshow('Raspbian Camera Stream', frame)

    # Esci dal ciclo se si preme il tasto 'q'
    # if keyboard.is_pressed('q'):
    #   print('Visione terminata')
    #   picam2.stop()
    #   stop_event1.set()

    except KeyboardInterrupt:
        print("Interruzione")
        picam2.stop()
    finally:
        picam2.stop()


def genera_angolo(stop_event1):
    global angolo1
    global angolo2
    global angolo3
    global lock
    while not stop_event1.is_set():
        angolo1 = randint(0, 180)
        angolo2 = randint(0, 180)
        angolo3 = randint(0, 180)
        # print("Angolo 1 = "+str(angolo1))
        # print("Angolo 2 = "+str(angolo2))
        # print("Angolo 3 = "+str(angolo3))
        time.sleep(0.1)
        # if keyboard.is_pressed("q"):
        #     print("Motori fermati")
        #     picam2.stop()
        #     stop_event1.set()


def setup_servo():
    pulse = 500 + (32 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    pulse1 = 500 + (29 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
    pulse2 = 500 + (30 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN2, pulse2)
    time.sleep(2)


def stopServo():
    # pi.set_servo_pulsewidth(SERVO_PIN, 0)
    # pi.set_servo_pulsewidth(SERVO_PIN1, 0)
    # pi.set_servo_pulsewidth(SERVO_PIN2, 0)
    # pi.stop()
    print("Gestione motori terminata")



def setAngle():
    try:
        while True:
            #print("Aggiorno ")
            # angolo = input()
            # angolo = int(angolo)
            if velocita < 25000:
                #print("Motore 1 = "+str(t1))
                #print("Motore 2 = "+str(t2))
                #print("Motore 3 = "+str(t3))
                
                pulse = 500 + ((t1+2) * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN, pulse)
                pulse1 = 500 + ((t2) * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
                pulse2 = 500 + ((t3) * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN2, pulse2)
                # print(angolo)
                time.sleep(t)####################################
                #pi.set_servo_pulsewidth(SERVO_PIN, 0)
                #pi.set_servo_pulsewidth(SERVO_PIN1, 0)
                #pi.set_servo_pulsewidth(SERVO_PIN2, 0)
                
            #     if keyboard.is_pressed("q"):
            #         setup_servo()
            #         stopServo()
            #         break
            # if keyboard.is_pressed("q"):
            #     setup_servo()
            #     stopServo()
            #     break
    except KeyboardInterrupt:
        print("Interruzione")
    finally:
        setup_servo()
        pi.set_servo_pulsewidth(SERVO_PIN, 35)
        pi.set_servo_pulsewidth(SERVO_PIN1, 35)
        pi.set_servo_pulsewidth(SERVO_PIN2, 35)


def calibrate_servo():
    print("Inserisci angolo 1")
    angle_1 = int(input())
    print("Inserisci angolo 2")
    angle_2 = int(input())
    print("Inserisci angolo 3")
    angle_3 = int(input())
    pulse = 500 + (angle_1 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    pulse1 = 500 + (angle_2 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
    pulse2 = 500 + (angle_3 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN2, pulse2)
    print(angle_1)
    print(angle_2)
    print(angle_3)
    time.sleep(2)
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.set_servo_pulsewidth(SERVO_PIN1, 0)
    pi.set_servo_pulsewidth(SERVO_PIN2, 0)


def muovi_avanti(servo, min, max):
    pulse = 500 + (min * 2000 / 180)
    pi.set_servo_pulsewidth(servo, pulse)
    for angle in range(min, max + 1):
        pulse = 500 + (angle * 2000 / 180)
        pi.set_servo_pulsewidth(servo, pulse)
        time.sleep(0.05)


def muovi_indietro(servo, min, max):
    pulse = 500 + (max * 2000 / 180)
    pi.set_servo_pulsewidth(servo, pulse)
    for angle in range(max, min - 1, -1):
        pulse = 500 + (angle * 2000 / 180)
        pi.set_servo_pulsewidth(servo, pulse)
        time.sleep(0.05)


def test_servo(ang_min, ang_max):
    pulse = 500 + (ang_min * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    pulse = 500 + (ang_max * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN1, pulse)
    pulse = 500 + (ang_max * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN2, pulse)
    time.sleep(1)

    # Inizio

    thread = threading.Thread(
        target=muovi_avanti,
        args=(
            SERVO_PIN,
            ang_min,
            ang_max,
        ),
    )
    thread1 = threading.Thread(
        target=muovi_indietro,
        args=(
            SERVO_PIN1,
            ang_min,
            ang_max,
        ),
    )
    thread.start()
    thread1.start()
    thread.join()
    thread1.join()

    thread1 = threading.Thread(
        target=muovi_avanti,
        args=(
            SERVO_PIN1,
            ang_min,
            ang_max,
        ),
    )
    thread2 = threading.Thread(
        target=muovi_indietro,
        args=(
            SERVO_PIN2,
            ang_min,
            ang_max,
        ),
    )
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()

    thread = threading.Thread(
        target=muovi_indietro,
        args=(
            SERVO_PIN,
            ang_min,
            ang_max,
        ),
    )
    thread2 = threading.Thread(
        target=muovi_avanti,
        args=(
            SERVO_PIN2,
            ang_min,
            ang_max,
        ),
    )
    thread.start()
    thread2.start()
    thread.join()
    thread2.join()


# muovi_servo()
# print("CI SIAMO")
# manualSetAngle()
# setAngle()
# pwm.stop()

opzione = 10
while opzione != 0:
    setup_servo()
    print("Premi 1 per avviare il ball balancer")
    print("Premi 2 per calibrare i servo")
    print("Premi 3 per testare i servo")
    print("Premi 0 per uscire")
    temp = input()
    opzione = int(temp[-1])
    print(opzione)
    if opzione == 1:
        print("Opzione 1")
        #thread = threading.Thread(target=genera_angolo, args=(stop_event1,))
        thread1 = threading.Thread(target=vision, args=(stop_event1,))
        # thread.start()
        thread1.start()
        print("Thread avviato")
        setAngle()
        # thread.join()
        thread1.join()
    elif opzione == 2:
        calibrate_servo()
        setup_servo()
    elif opzione == 3:
        print("Inserisci angolo minimo di test")
        servo_min = int(input())
        print("Inserisci angolo massimo di test")
        servo_max = int(input())
        test_servo(servo_min, servo_max)
        setup_servo()
    elif opzione == 0:
        pi.stop()
