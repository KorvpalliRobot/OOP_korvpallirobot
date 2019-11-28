import time
from threading import Thread

import cv2
import numpy as np
import pyrealsense2 as rs


class ImageCapCV:

    def command_thread(self):
        while self.running:
            (self.grabbed, self.currentFrame) = self.camera.read()
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.running = False

    def __init__(self, src=0):
        self.running = True
        self.grabbed = None
        self.camera = cv2.VideoCapture(src)
        self.currentFrame = self.camera.read()
        Thread(name="commandThread", target=self.command_thread).start()

    def get_frame(self):
        return self.currentFrame


class ImageCapRS2:

    def command_thread(self):
        while self.running:
            self.frames = self.pipeline.wait_for_frames()
            # aligned_frames = rs.align(rs.stream.color).process(frames)
            self.depth_frame = self.frames.get_depth_frame()
            #self.depth_frame = np.asanyarray(self.depth.get_data())
            color_frame = self.frames.get_color_frame()
            self.current_frame = np.asanyarray(color_frame.get_data())
            if self.stop_flag.is_set():
                self.pipeline.stop()
                self.running = False

    def __init__(self, stop_flag, src=0):
        self.running = True
        self.current_frame = None
        self.depth_frame = None
        self.frames = None
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            sensors = dev.query_sensors()
            for sensor in sensors:
                if sensor.is_depth_sensor():
                    sensor.set_option(rs.option.enable_auto_exposure, 0)

        self.pipeline.start()
        self.stop_flag = stop_flag
        Thread(name="commandThread", target=self.command_thread).start()
        time.sleep(1.5)

    def get_frames(self):
        return self.frames

    def get_frame(self):
        return self.current_frame

    def get_depth_frame(self):
        return self.depth_frame


class Camera:
    def __init__(self, basket, balls, camera_thread, stop_flag):
        self.cap = cv2.VideoCapture(1)
        # self.cap.set(3, 1280)
        # self.cap.set(4, 720)
        self.kernel = 3
        self.morph = np.ones((10, 10), np.uint8)
        self.basket = basket
        self.balls = balls
        self.thresh_min_balls = balls.thresh_min_limits
        self.thresh_max_balls = balls.thresh_max_limits
        self.thresh_min_basket = basket.thresh_min_limits
        self.thresh_max_basket = basket.thresh_max_limits
        self.previous_time = time.time()
        self.current_time = time.time()
        self.camera_thread = camera_thread

        self.stop_flag = stop_flag

        # Detector configuration
        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.minArea = 8
        self.blobparams.maxArea = 1000000
        self.blobparams.filterByColor = True
        self.blobparams.filterByCircularity = False
        self.blobparams.blobColor = 255
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)

    def find_objects(self):
        camera = self

        # Check for stop signals
        frame = camera.camera_thread.get_frame()
        frame = cv2.medianBlur(frame, 3)

        # print("Distance:", self.camera_thread.get_depth().get_distance(320, 10))

        width = len(frame[0])
        img_center = width / 2
        img_height = len(frame)
        # print(width)

        thresholded_balls = camera.thresholding(frame, camera.thresh_min_balls, camera.thresh_max_balls)
        thresholded_basket = camera.thresholding(frame, camera.thresh_min_basket, camera.thresh_max_basket)

        # print(camera.thresh_min_basket, camera.thresh_max_basket)

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
        frame, basket_x, diameter, extreme_points = self.find_contours(frame, thresholded_basket)

        # The basket's x-coordinate and diameter (for distance calculations)
        # Width variable for remembering the last true x-coordinate (when not seeing basket)
        self.basket.set_x(basket_x, width)
        self.basket.set_diameter(diameter)
        self.basket.set_extreme_points(extreme_points)
        # print("d:", diameter)

        # Draw a vertical line at the center of the image (for troubleshooting)
        frame = self.draw_centerline_on_frame(frame, width, img_height)

        cv2.imshow('Thresh Ball', thresholded_balls)
        cv2.imshow('Thresh Basket', thresholded_basket)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_flag.set()
            cv2.destroyAllWindows()
            # return img_center, img_height

        return img_center, img_height

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

    def get_distance_to_basket(self, n=5):
        cumulative_distance = 0
        for i in range(n):
            depth_frame = rs.align(rs.stream.color).process(self.camera_thread.get_frames()).get_depth_frame()
            extreme_points = self.basket.get_extreme_points()  # [self.l_m, self.r_m, self.t_m, self.b_m]
            y2 = extreme_points[3]
            # y1 = extreme_points[2]
            y1 = y2 - 20
            x1 = extreme_points[0]
            x2 = extreme_points[1]
            distance = []
            for y in range(y1, y2):
                for x in range(x1, x2):
                    distance.append(depth_frame.get_distance(x, y))

            try:
                distance.sort()
                avg_distance = distance[len(distance) // 2]
                if avg_distance <= 2 and cumulative_distance == 0 or n == 1:
                    return avg_distance
                cumulative_distance += avg_distance
                time.sleep(0.05)
            except ZeroDivisionError:
                print("Error when calculating basket distance. Zero division!")
                return -1
            except IndexError:
                print("Error when calculating basket distance. Index error!")
                return -1
        return cumulative_distance / n

    def get_more_percise_distance_to_basket(self, iterations=3):
        culmulative_distance = 0
        for i in range(iterations):
            culmulative_distance += self.get_distance_to_basket()
        return culmulative_distance/iterations

    @staticmethod
    def median_from_unsorted_array(array):
        median_pos = len(array) // 2

        while len(array) > 1:
            pivot = array[len(array) // 2]

            left = []
            equal = []
            right = []
            for element in array:
                if element > pivot:
                    right.append(element)
                elif element < pivot:
                    left.append(element)
                else:
                    equal.append(element)

            if len(left) - 1 < median_pos < len(left) + len(equal):
                return equal[0]
            else:
                if len(left) > len(right) + len(equal):
                    array = left
                    median_pos = median_pos
                elif len(right) > len(left) + len(equal):
                    array = right
                    median_pos = median_pos - (len(left) + len(equal))
                else:
                    return equal[0]
        if len(array) == 0:
            return -1
        return array[0]

    @staticmethod
    def draw_centerline_on_frame(frame, frame_width, frame_height):
        # x1 = frame center; y1: frame height (top); x2: frame center; y2: frame height (bottom).
        x = frame_width // 2
        y1 = frame_height
        y2 = 0
        # Set line attributes.
        line_thickness = 1
        line_color = (255, 0, 0)
        cv2.line(frame, (x, y1), (x, y2), line_color, line_thickness)

        return frame

    @staticmethod
    def find_contours(frame, thresholded):
        contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cx = 0
        diameter = 0
        sorted_contours = sorted(contours, key=cv2.contourArea)
        extreme_points = [0, 0, 0, 0]

        min_area = 300

        if len(sorted_contours) > 0:
            for i in range(1, len(sorted_contours)):
                if cv2.contourArea(sorted_contours[-1 * i]) > min_area:
                    cv2.drawContours(frame, sorted_contours[-1], -1, (0, 255, 0), 3)
                    break

        try:
            if len(sorted_contours) > 0:
                if cv2.contourArea(sorted_contours[-1]) > min_area:
                    m = cv2.moments(sorted_contours[-1])
                    # print(m.keys())

                    # The centroid point
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    # print(cx)

                    # The extreme points

                    l_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmin()][0])[0]
                    r_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmax()][0])[0]
                    t_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 1].argmin()][0])[1]
                    b_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 1].argmax()][0])[1]

                    extreme_points = [l_m, r_m, t_m, b_m]

                    diameter = r_m - l_m
                    # print("Diameter:", diameter)

                    # for contour in contours:
                    #     cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
        except:
            cx = 0

        return frame, cx, diameter, extreme_points

