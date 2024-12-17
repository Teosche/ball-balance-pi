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
        Cattura un frame e lo restituisce in formato JPEG.
        """
        frame = self.camera.capture_array()
        # Annotazione (opzionale)
        text = "Streaming video"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Converte il frame in JPEG
        _, buffer = cv2.imencode(".jpg", frame)
        return buffer.tobytes()
