from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import time

camera = PiCamera()

camera.resolution = (320, 320)
camera.framerate = 32

time.sleep(0.1)

raw_capture = PiRGBArray(camera, size=(640, 480))

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):

    image = frame.array

    cv2.imshow("Frame", image)

    key = cv2.waitKey(1) & 0xFF

    raw_capture.truncate(0)

    if key == ord("q"):
        break
