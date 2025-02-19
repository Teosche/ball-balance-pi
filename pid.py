import numpy as np


class PID:
    """
    A PID controller with anti-windup, derivative filtering, and predictive control
    for smoother and more stable adjustments.
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: tuple):
        """
        Initialize the PID controller with improvements.

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
        self.previous_derivative = (0, 0)  # For derivative filtering
        self.dt_min = 0.01  # Minimum dt to prevent division errors
        self.max_integral = 10  # Anti-windup limit

    def update(self, feedback_value: tuple, dt: float) -> tuple:
        """
        Update the PID output based on the current feedback value.

        Args:
            feedback_value (tuple): Current coordinates (x, y).
            dt (float): Time step since the last update.

        Returns:
            tuple: PID control output (x, y) as floats.
        """
        dt = max(dt, self.dt_min)

        error_x = self.setpoint[0] - feedback_value[0]
        error_y = self.setpoint[1] - feedback_value[1]

        self.integral = (
            np.clip(
                self.integral[0] + error_x * dt, -self.max_integral, self.max_integral
            ),
            np.clip(
                self.integral[1] + error_y * dt, -self.max_integral, self.max_integral
            ),
        )

        alpha = 0.8
        derivative_x = (
            alpha * (error_x - self.previous_error[0]) / dt
            + (1 - alpha) * self.previous_derivative[0]
        )
        derivative_y = (
            alpha * (error_y - self.previous_error[1]) / dt
            + (1 - alpha) * self.previous_derivative[1]
        )

        prediction_x = error_x + derivative_x * dt * 0.5
        prediction_y = error_y + derivative_y * dt * 0.5

        output_x = (
            self.kp * prediction_x + self.ki * self.integral[0] + self.kd * derivative_x
        )
        output_y = (
            self.kp * prediction_y + self.ki * self.integral[1] + self.kd * derivative_y
        )

        self.previous_error = (error_x, error_y)
        self.previous_derivative = (derivative_x, derivative_y)

        return output_x, output_y
