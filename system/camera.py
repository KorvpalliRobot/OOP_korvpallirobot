import cv2
import numpy as np
import xbee


class Camera:
    def __init__(self, basket, balls):
        self.cap = cv2.VideoCapture(1)
        self.kernel = 7
        self.morph = np.ones((7, 7), np.uint8)
        self.basket = basket
        self.balls = balls

    def find_objects(self, xbee):
        camera = self
        while xbee.stop_flag == False:
            ret, frame = camera.cap.read()
            thresholded_balls = camera.thresholding(frame, camera.balls)
        print("Camera.find_objects terminated!")

    def thresholding(self, frame, object):
        # RGB to HSV colour space
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # BLUR averaging
        frame = cv2.blur(frame, (self.kernel, self.kernel))

        # Operations on the frame
        thresholded = cv2.inRange(frame, object.thresh_min_limits, object.thresh_max_limits)

        thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, self.morph)

        return thresholded
