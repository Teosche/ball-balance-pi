import math
import time
import numpy as np
from picamera2 import Picamera2
import cv2


class Camera:
    def __init__(self, resolution=(640, 480)):
        self.camera = Picamera2()
        self.camera_config = self.camera.create_preview_configuration(
            main={"size": resolution}
        )
        self.camera.configure(self.camera_config)
        self.camera.start()

    def get_frame(self):
        """
        Frame capture.
        """
        frame = self.camera.capture_array()
        text = "Streaming video"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # filters
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurFrame = cv2.GaussianBlur(gray, (17, 17), 0)

        # init variablesto neet fix
        global x
        global y
        global speed
        lock = True
        previous_x = 10
        previous_y = 120

        # define circle contours
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
        circles = np.round(circles[0, :]).astype("int")

        for x, y, r in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            x = round(x / 2)
            y = round(y / 2)
            if lock is True:
                previous_x = x
                previous_y = y
                lock = False

            module = math.sqrt((x - previous_x) ** 2 + (y - previous_y) ** 2)
            speed = round(module, 1)
            print(f"Position: (X:{x}: Y:{y})")
            print(speed)
            previous_x = round(x / 2)
            previous_y = round(y / 2)

            time.sleep(0.1)

        _, buffer = cv2.imencode(".jpg", frame)
        return buffer.tobytes()
