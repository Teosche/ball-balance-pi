import RPi.GPIO as GPIO
import time

SERVO_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.stop()
pwm.start(0)

def setAngle():
    angle=0
    pwm.ChangeDutyCycle(0)
    time.sleep(6)
    if angle == 0:
       for angle in range(0,181,+1):
          duty = 2+ (angle/18)
          
          pwm.ChangeDutyCycle(duty)
          print(angle)
          time.sleep(0.5)
    if angle == 12:
       for angle in range(180,-1, -1):
          duty = 2+ (angle/18)
          
          pwm.ChangeDutyCycle(duty)
          print(angle)
          time.sleep(0.5)
    
setAngle()
pwm.stop()
GPIO.cleanup()
