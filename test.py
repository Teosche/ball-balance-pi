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
print("Caricamento...")
pi = pigpio.pi()
if not pi.connected:
    print("Errore pigpio")
    exit(1)
angolo1 = 0
angolo2 = 0
angolo3 = 0


t = 0.3
posizione_x = 0
posizione_y = 0
velocita = 0

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (240, 240)})
picam2.configure(camera_config)

picam2.start()


stop_event1 = threading.Event()


def vision(stop_event1):
    try:
        global posizione_x
        global posizione_y
        lock = True
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
                minRadius=2,
                maxRadius=40,
            )

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for x, y, r in circles:
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
                    posizione_x = round(x / 2)
                    posizione_y = round(y / 2)
                    if lock is True:
                        posizione_x_prec = posizione_x
                        posizione_y_prec = posizione_y
                        lock = False

                    mod = math.sqrt(
                        (posizione_x - posizione_x_prec) ** 2
                        + (posizione_y - posizione_y_prec) ** 2
                    )
                    velocita = round(mod, 1)
                    print(f"Posizione: (X:{posizione_x}: Y:{posizione_y})")
                    print(velocita)
                    posizione_x_prec = round(x / 2)
                    posizione_y_prec = round(y / 2)

                    time.sleep(t)
            else:
                velocita = 0
                print("Niente sfera")

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
        time.sleep(0.2)
        # if keyboard.is_pressed("q"):
        #     print("Motori fermati")
        #     picam2.stop()
        #     stop_event1.set()


def setup_servo():
    pulse = 500 + (50 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse)
    pulse1 = 500 + (50 * 2000 / 180)
    pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
    pulse2 = 500 + (50 * 2000 / 180)
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
            # print("Aggiorno ")
            # angolo = input()
            # angolo = int(angolo)
            if velocita > 2.5:
                pulse = 500 + (angolo1 * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN, pulse)
                pulse1 = 500 + (angolo2 * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN1, pulse1)
                pulse2 = 500 + (angolo3 * 2000 / 180)
                pi.set_servo_pulsewidth(SERVO_PIN2, pulse2)
                # print(angolo)
                time.sleep(0.5)
                pi.set_servo_pulsewidth(SERVO_PIN, 0)
                pi.set_servo_pulsewidth(SERVO_PIN1, 0)
                pi.set_servo_pulsewidth(SERVO_PIN2, 0)
                time.sleep(2)
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
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_PIN1, 0)
        pi.set_servo_pulsewidth(SERVO_PIN2, 0)


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
        # thread = threading.Thread(target=genera_angolo, args=(stop_event1,))
        thread1 = threading.Thread(target=vision, args=(stop_event1,))
        # thread.start()
        thread1.start()
        # setAngle()
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
