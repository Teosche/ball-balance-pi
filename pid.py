class PIDController:
    """
    A PID controller for 2D coordinates (x, y).

    The controller calculates the proportional, integral, and derivative
    outputs based on the feedback value and a given setpoint.

    Attributes:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (tuple): Target coordinates (x, y).
        previous_error (tuple): Previous error values for (x, y).
        integral (tuple): Integral term accumulator for (x, y).
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: tuple):
        """
        Initialize the PID controller.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (tuple): Target coordinates (x, y).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.previous_error = (0, 0)
        self.integral = (0, 0)

    def update(self, feedback_value: tuple, dt: float) -> tuple:
        """
        Update the PID output based on the current feedback value.

        Args:
            feedback_value (tuple): Current coordinates (x, y).
            dt (float): Time step since the last update.

        Returns:
            tuple: PID control output (x, y) as floats.
        """
        error_x = self.setpoint[0] - feedback_value[0]
        error_y = self.setpoint[1] - feedback_value[1]

        self.integral = (
            self.integral[0] + error_x * dt,
            self.integral[1] + error_y * dt,
        )

        derivative_x = (error_x - self.previous_error[0]) / dt if dt > 0 else 0
        derivative_y = (error_y - self.previous_error[1]) / dt if dt > 0 else 0

        output_x = (
            self.kp * error_x + self.ki * self.integral[0] + self.kd * derivative_x
        )
        output_y = (
            self.kp * error_y + self.ki * self.integral[1] + self.kd * derivative_y
        )

        self.previous_error = (error_x, error_y)

        return output_x, output_y
