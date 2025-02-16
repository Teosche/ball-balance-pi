import time


class Servo:
    """
    A class to manage servo motors.
    """

    def __init__(self, pi, pin1=18, pin2=23, pin3=24):
        """
        Initialize servos with pigpio.

        Args:
            pi (pigpio.pi): pigpio instance
            pin1 (int): Servo pin 1
            pin2 (int): Servo pin 2
            pin3 (int): Servo pin 3
        """
        self.pi = pi
        self.pins = [pin1, pin2, pin3]
        self.reset_servo()

    def set_pulse(self, theta):
        """
        Convert degrees to pulse width.
        """
        return 500 + ((theta) * 2000 / 180)

    def move_servos(self, angles):
        """
        Move the servos to the specified angles.
        Args:
            angles (tuple): (angle1, angle2, angle3)
        """
        for pin, angle in zip(self.pins, angles):
            pulse = self.set_pulse(angle)
            self.pi.set_servo_pulsewidth(pin, pulse)

    def reset_servo(self):
        """
        Reset servos to default position.
        """
        self.move_servos((30, 30, 30))
        time.sleep(2)
