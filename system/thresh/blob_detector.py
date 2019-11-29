import queue
import threading
from threading import Event

import system.mainboard as mainb
import system.remote_control as remote_control
import system.robot as r

import numpy as np
import cv2
import time

# open the camera
from system.balls import Balls
from system.basket import Basket
from system.camera import Camera
from system.camera import ImageCapRS2

stop_flag = Event()
stop_flag.clear()
autonomy = Event()
autonomy.clear()
basket = Basket("thresh_basket_blue.txt")
balls = Balls("thresh_ball.txt")
camera_thread = ImageCapRS2(stop_flag)
camera = Camera(basket, balls, camera_thread, stop_flag)
# Mainboard communication
mainboard = mainb.Mainboard(autonomy, stop_flag)
thread_mainboard = threading.Thread(name="mainboard", target=mainboard.send_to_mainboard, daemon=True)
thread_mainboard.start()

robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)

# Manual control
q_thrower_speed = queue.Queue()
thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(mainboard, autonomy, stop_flag, q_thrower_speed), daemon=True)
thread_manual_control.start()

# Set the initial time
aeg = time.time()

# set the kernel size for blurring
kernel = 3

# set the kernel size for morphology, the first is a matrix and the second is an integer
morph = np.ones((10, 10), np.uint8)
morphvalue = 10

# Selector to choose whether to update the threshold values for the ball or the basket.
selector = 0

# Read global variables for trackbars from thresh.txt
def read_values(filename) :
    file = open(filename)
    f = list(file)
    f = [x.strip() for x in f]
    lH = int(f[0])
    lS = int(f[1])
    lV = int(f[2])
    hH = int(f[3])
    hS = int(f[4])
    hV = int(f[5])
    file.close()
    return [lH, lS, lV, hH, hS, hV]

filename = "thresh_ball.txt"
values = read_values(filename)
lH = values[0]
lS = values[1]
lV = values[2]
hH = values[3]
hS = values[4]
hV = values[5]

""" igaks juhuks vanad väärtused:
    lH = 0
    lS = 242
    lV = 131
    hH = 34
    hS = 255
    hV = 255
"""


def update_all_limits(filename):
    global lH, lS, lV, hH, hS, hV
    values = read_values(filename)
    lH = values[0]
    lS = values[1]
    lV = values[2]
    hH = values[3]
    hS = values[4]
    hV = values[5]

    # Attach a trackbar to a window
    cv2.createTrackbar("P|S|R=", "Trackbars", selector, 2, update_selector)
    cv2.createTrackbar("lH", "Trackbars", lH, 255, updatelH)
    cv2.createTrackbar("lS", "Trackbars", lS, 255, updatelS)
    cv2.createTrackbar("lV", "Trackbars", lV, 255, updatelV)
    cv2.createTrackbar("hH", "Trackbars", hH, 255, updatehH)
    cv2.createTrackbar("hS", "Trackbars", hS, 255, updatehS)
    cv2.createTrackbar("hV", "Trackbars", hV, 255, updatehV)

    # Trackbar for blur kernel size
    cv2.createTrackbar("Blur kernel size", "Trackbars", kernel, 19, updatekernel)
    cv2.createTrackbar("Morph kernel size", "Trackbars", morphvalue, 19, updatemorph)

# A callback function for a trackbar
# It is triggered every time the slider on trackbar is used
def updatelH(new_value):
    # make sure to write the new value into the global variable
    global lH
    lH = new_value
    return


def updatelS(new_value):
    # make sure to write the new value into the global variable
    global lS
    lS = new_value
    return


def updatelV(new_value):
    # make sure to write the new value into the global variable
    global lV
    lV = new_value
    return


def updatehH(new_value):
    # make sure to write the new value into the global variable
    global hH
    hH = new_value
    return


def updatehS(new_value):
    # make sure to write the new value into the global variable
    global hS
    hS = new_value
    return


def updatehV(new_value):
    # make sure to write the new value into the global variable
    global hV
    hV = new_value
    return


def updatekernel(new_value):
    # make sure to write the new value into the global variable
    if new_value % 2 != 0:
        global kernel
        kernel = new_value
    else:
        kernel = new_value + 1
    return


# updatemorph updates the both integer "morphvalue" for trackbars and the matrix for morphology
def updatemorph(new_value):
    # make sure to write the new value into the global variable
    if new_value % 2 != 0:
        global morph
        morph = np.ones((new_value, new_value), np.uint8)
        global morphvalue
        morphvalue = new_value
    else:
        morph = np.ones((new_value + 1, new_value + 1), np.uint8)
        morphvalue = new_value + 1
    return


# Selector to choose whether to update the threshold values for the ball or the basket.
# 0 == ball; 1 == basket.
# This is a function to update the corresponding trackbar.
def update_selector(new_value):
    # make sure to write the new value into the global variable
    global selector
    selector = new_value
    return

# Create a window
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Trackbars", 1280, 480)
# Attach a trackbar to a window
cv2.createTrackbar("P|S|R=", "Trackbars", selector, 2, update_selector)
cv2.createTrackbar("lH", "Trackbars", lH, 255, updatelH)
cv2.createTrackbar("lS", "Trackbars", lS, 255, updatelS)
cv2.createTrackbar("lV", "Trackbars", lV, 255, updatelV)
cv2.createTrackbar("hH", "Trackbars", hH, 255, updatehH)
cv2.createTrackbar("hS", "Trackbars", hS, 255, updatehS)
cv2.createTrackbar("hV", "Trackbars", hV, 255, updatehV)

# Trackbar for blur kernel size
cv2.createTrackbar("Blur kernel size", "Trackbars", kernel, 19, updatekernel)
cv2.createTrackbar("Morph kernel size", "Trackbars", morphvalue, 19, updatemorph)

# Detector configuration
blobparams = cv2.SimpleBlobDetector_Params()
blobparams.minArea = 10
blobparams.maxArea = 1000000
blobparams.filterByColor = True
blobparams.filterByCircularity = False
blobparams.blobColor = 255
detector = cv2.SimpleBlobDetector_create(blobparams)


def find_contours(frame, thresholded):
    contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    black_listed = []
    cx = 0
    diameter = 0

    sorted_contours = sorted(contours, key=cv2.contourArea)

    if len(sorted_contours) > 0:
        for i in range(1, len(sorted_contours)):
            if cv2.contourArea(sorted_contours[-1 * i]) > 3:
                cv2.drawContours(frame, sorted_contours[-1], -5, (0, 255, 0), 3)
                break

    try:
        if len(sorted_contours) > 0:
            # image moment
            for i in range(1, len(sorted_contours)):
                if cv2.contourArea(sorted_contours[-1 * i]) < 3:
                    continue
                m = cv2.moments(sorted_contours[-1*i])
                # print(m.keys())

                # The centroid point
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                # print(cx)

                # The extreme points
                l_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmin()][0])[0]
                r_m = tuple(sorted_contours[-1][sorted_contours[-1][:, :, 0].argmax()][0])[0]

                diameter = r_m - l_m
                break
                # print("Diameter:", diameter)

                # for contour in contours:
                #     cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
    except:
        cx = 0

    return frame, thresholded


def blob_detection(frame, thresholded):
    keypoints = detector.detect(thresholded)
    frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for keypoint in keypoints:
        x = int(keypoint.pt[0])
        y = int(keypoint.pt[1])
        tekst = "x: " + str(x) + " y: " + str(y)
        cv2.putText(frame, tekst, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return frame, thresholded

while True:
    # read the image from the camera
    frame = camera.camera_thread.get_frame()
    camera.find_objects()

    w = camera.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = camera.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    #print("Width:", w, "; Height:", h)

    # Print values from stereo cameras.
    depth_frame = camera.camera_thread.get_depth_frame()
    depth = np.asanyarray(depth_frame.get_data())

    #print("Distance=", camera.get_distance_to_basket(n=1))

    # RGB to HSV colour space
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # BLUR averaging
    frame = cv2.blur(frame, (kernel, kernel))

    # BLUR Bilateral
    # frame = cv2.bilateralFilter(frame,kernel,75,75)

    # BLUR Gaussian
    # frame = cv2.GaussianBlur(frame, (kernel, kernel), 0)

    lowerLimits = np.array([lH, lS, lV])
    upperLimits = np.array([hH, hS, hV])

    # Our operations on the frame come here
    thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
    # thresholded = cv2.bitwise_not(thresholded)

    # Morphology
    # thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, morph)
    thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, morph)
    # thresholded = cv2.erode(thresholded,morph,iterations = 1)

    #outimage = cv2.bitwise_and(frame, frame, mask=thresholded)

    # Write the framerate
    eelmine_aeg = aeg
    aeg = time.time()
    framerate = 1 / (aeg - eelmine_aeg)
    cv2.putText(frame, str(framerate), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    if selector == 0:
        if filename != "thresh_ball.txt":
            filename = "thresh_ball.txt"
            update_all_limits(filename)

        # BLOB DETECTION
        frame, thresholded = blob_detection(frame, thresholded)
    elif selector == 1:
        if filename != "thresh_basket_blue.txt":
            filename = "thresh_basket_blue.txt"
            update_all_limits(filename)
        frame, thresholded = find_contours(frame, thresholded)
    else:
        if filename != "thresh_basket_pink.txt":
            filename = "thresh_basket_pink.txt"
            update_all_limits(filename)
        frame, thresholded = find_contours(frame, thresholded)

    cv2.imshow('Original', frame)
    cv2.imshow('Thresh', thresholded)
    #cv2.imshow('Depth', depth_colormap)

    # Display the resulting frame
    #cv2.imshow('Processed', outimage)

    # Quit the program when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop_flag.set()
        break

# Writing threshold variables to file
# Again, variable "selector" chooses which file to write.
if selector == 0:
    f = open("thresh_ball.txt", "w")
elif selector == 1:
    f = open("thresh_basket_blue.txt", "w")
else:
    f = open("thresh_basket_pink.txt", "w")

f.write(str(lH) + "\n")
f.write(str(lS) + "\n")
f.write(str(lV) + "\n")
f.write(str(hH) + "\n")
f.write(str(hS) + "\n")
f.write(str(hV) + "\n")
f.close()

# When everything done, release the capture
print('closing program')
cv2.destroyAllWindows()

thread_manual_control.join(timeout=0.5)
thread_mainboard.join(timeout=0.5)
