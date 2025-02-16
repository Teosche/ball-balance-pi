import math
import numpy as np
from picamera2 import Picamera2
import cv2


class Camera:
    """
    A class to handle camera operations, including capturing frames, detecting circles,
    calculating the speed of detected objects, and rendering overlays on the frames.
    """

    def __init__(self):
        """
        Initialize the camera and its configuration.
        """
        self.camera = Picamera2()
        self.camera_config = self.camera.create_preview_configuration()
        self.camera.configure(self.camera_config)
        self.camera.start()

        self.previous_x = None
        self.previous_y = None
        self.speed = 0
        self.circle = None

    def get_frame(self) -> np.ndarray:
        """
        Capture the current frame, detect circles, calculate speed, and annotate the frame.

        Returns:
            np.ndarray: The encoded JPEG buffer of the processed frame.
        """
        frame = self.capture_frame()
        self.detect_circle(frame)

        if self.circle is not None:
            self.get_position_information(frame)
        else:
            print("No circles detected.")
            self.speed = 0

        _, buffer = cv2.imencode(".jpg", frame)
        return buffer.tobytes()

    def calculate_speed(self, x: int, y: int, previous_x: int, previous_y: int) -> int:
        """
        Calculate the speed of the detected object between two frames.
        Args:
            x (int): The current x-coordinate of the object.
            y (int): The current y-coordinate of the object.
            previous_x (int): The previous x-coordinate of the object.
            previous_y (int): The previous y-coordinate of the object.
        Returns:
            float: The calculated speed.
        """
        module = math.sqrt((x - previous_x) ** 2 + (y - previous_y) ** 2)
        return round(module)

    def get_position_information(self, frame):
        """
        Process detected circles to annotate the frame, calculate speed, and update position.

        Args:
            frame (np.ndarray): The current frame on which circles are detected and annotated.
        """
        self.circle = np.round(self.circle[0, :]).astype("int")

        for x, y, r in self.circle:
            self.print_circle(frame, x, y, r)
            if self.previous_x is None or self.previous_y is None:
                self.previous_x, self.previous_y = x, y
            self.speed = self.calculate_speed(x, y, self.previous_x, self.previous_y)
            self.previous_x, self.previous_y = x, y

    def capture_frame(self) -> np.array:
        """
        Capture a frame from the camera.

        Returns:
            np.array: The captured frame.
        """
        frame = self.camera.capture_array()
        text = "Streaming video"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return frame

    def detect_circle(self, frame):
        """
        Detect circles in the provided frame using the HoughCircles algorithm.

        Args:
            frame (np.array): The frame in which circles are to be detected.
        """
        blur_frame = cv2.GaussianBlur(
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (17, 17), 0
        )

        self.circle = cv2.HoughCircles(
            blur_frame,
            cv2.HOUGH_GRADIENT,
            1.2,
            100000,
            param1=100,
            param2=30,
            minRadius=2,
            maxRadius=50,
        )

    def print_circle(self, frame: np.array, x: int, y: int, r: int):
        """
        Draw a circle and a center point over the detected object.

        Args:
            frame (np.array): The frame to draw on.
            x (int): The x-coordinate of the circle's center.
            y (int): The y-coordinate of the circle's center.
            r (int): The radius of the circle.
        """
        cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
