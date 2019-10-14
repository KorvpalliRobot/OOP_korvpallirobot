import cv2
import numpy as np
import time


class Camera:
    def __init__(self, basket, balls, stop_flag):
        self.cap = cv2.VideoCapture(1)
        self.kernel = 7
        self.morph = np.ones((7, 7), np.uint8)
        self.basket = basket
        self.balls = balls
        self.thresh_min_balls = balls.thresh_min_limits
        self.thresh_max_balls = balls.thresh_max_limits
        self.thresh_min_basket = basket.thresh_min_limits
        self.thresh_max_basket = basket.thresh_max_limits
        self.previous_time = time.time()
        self.current_time = time.time()

        self.stop_flag = stop_flag

        # Detector configuration
        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.minArea = 5
        self.blobparams.maxArea = 1000000
        self.blobparams.filterByColor = True
        self.blobparams.filterByCircularity = False
        self.blobparams.blobColor = 255
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)

    def find_objects(self):
        camera = self

        # Check for stop signals
        if camera.stop_flag.is_set():
            camera.cap.release()
            cv2.destroyAllWindows()
            # When everything done, release the capture
            print("Camera.find_objects terminated!")
            return

        ret, frame = camera.cap.read()
        thresholded_balls = camera.thresholding(frame, camera.thresh_min_balls, camera.thresh_max_balls)
        thresholded_basket = camera.thresholding(frame, camera.thresh_min_basket, camera.thresh_max_basket)

        #print(camera.thresh_min_basket, camera.thresh_max_basket)

        # FPS
        self.previous_time = camera.current_time
        self.current_time = time.time()
        fps = int(1 / (camera.current_time - camera.previous_time))
        cv2.putText(frame, str(fps), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Operations concerning the ball.
        # Retrieve all ball keypoints
        frame, keypoints = self.blob_detector(frame, thresholded_balls)

        # Pass the keypoints to Balls instance, which sorts them etc
        self.balls.set_balls(keypoints)

        # Operations concerning the basket.
        frame, basket_x, diameter = self.find_contours(frame, thresholded_basket)

        # The basket's x-coordinate and diameter (for distance calculations)
        self.basket.set_x(basket_x)
        self.basket.set_diameter(diameter)
        #print(diameter)

        # Draw a vertical line at the center of the image (for troubleshooting)
        frame = self.draw_centerline_on_frame(frame, self.cap)

        cv2.imshow('Frame', frame)
        cv2.imshow('Thresh Ball', thresholded_balls)
        cv2.imshow('Thresh Basket', thresholded_basket)

        # Quit the program when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_flag.set()
            self.cap.release()
            cv2.destroyAllWindows()
            # When everything done, release the capture
            print("Camera.find_objects terminated!")
            return

    def thresholding(self, frame, thresh_min_limits, thresh_max_limits):
        # RGB to HSV colour space
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # BLUR averaging
        frame = cv2.blur(frame, (self.kernel, self.kernel))

        # Operations on the frame
        thresholded = cv2.inRange(frame, thresh_min_limits, thresh_max_limits)

        thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, self.morph)
        return thresholded

    def blob_detector(self, frame, thresholded):
        # BLOB DETECTION
        keypoints = self.detector.detect(thresholded)
        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        if len(keypoints) > 0:
            keypoint_largest = keypoints[0]
            current_max_size = 0
            for keypoint in keypoints:
                if keypoint.size > current_max_size:
                    current_max_size = keypoint.size
                    keypoint_largest = keypoint

            x = int(keypoint_largest.pt[0])
            y = int(keypoint_largest.pt[1])
            tekst = "x: " + str(x) + " y: " + str(y)
            cv2.putText(frame, tekst, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return frame, keypoints

    @staticmethod
    def draw_centerline_on_frame(frame, cap):
        # x1 = frame center; y1: frame height (top); x2: frame center; y2: frame height (bottom).
        x1 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) // 2)
        y1 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        x2 = x1
        y2 = 0
        # Set line attributes.
        line_thickness = 1
        line_color = (255, 0, 0)
        cv2.line(frame, (x1, y1), (x2, y2), line_color, line_thickness)

        return frame

    @staticmethod
    def find_contours(frame, thresholded):
        contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cx = 0
        diameter = 0
        sorted_contours = sorted(contours, key=cv2.contourArea)

        if len(sorted_contours) > 0:
            cv2.drawContours(frame, sorted_contours[-1], -1, (0, 255, 0), 3)

        try:
            if len(sorted_contours) > 0:
                # image moment
                m = cv2.moments(sorted_contours[-1])
                # print(m.keys())

                # The centroid point
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                # print(cx)

                # The extreme points
                l_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmin()][0])[0]
                r_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmax()][0])[0]

                diameter = r_m - l_m
                #print("Diameter:", diameter)


                # for contour in contours:
                #     cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
        except:
            cx = 0

        return frame, cx, diameter
