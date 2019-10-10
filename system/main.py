import robot as r
import camera as cam
import basket as bask
import balls as ball
import threading
import remote_control
import camera
import cv2
import numpy as np
import time
import queue


def main():
    # DATA
    # Holds the information about game logic state (game or manual)
    autonomy = threading.Event()
    # By default this should be set so that the robot starts autonomously
    autonomy.set()
    # Stop signal for all threads
    stop_flag = threading.Event()
    stop_flag.clear()

    # OBJECTS
    basket = bask.Basket("thresh/thresh_basket.txt")
    balls = ball.Balls("thresh/thresh_ball.txt")

    #xbee = xb.Xbee()
    camera = cam.Camera(basket, balls, stop_flag)
    mainboard = r.Mainboard(autonomy, stop_flag)
    robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)

    # Returns the horizontal position of the ball
    #thread_image_processing = threading.Thread(name="img", target=camera.find_objects, daemon=True)
    # Returns motor speeds needed to rotate to ball
    thread_game_logic = threading.Thread(name="auto", target=robot.autopilot, daemon=True)
    # Controls all the motors
    thread_mainboard_comm = threading.Thread(name="comm", target=mainboard.send_to_mainboard, daemon=True)
    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(mainboard, autonomy, stop_flag), daemon=True)

    # Start the threads
    #thread_image_processing.start()
    thread_game_logic.start()
    thread_mainboard_comm.start()
    thread_manual_control.start()

    # The main loop for our program, use to display values etc
    while True:
        # Check for stop signals
        if stop_flag.is_set():

            #thread_image_processing.join()
            thread_game_logic.join()
            thread_mainboard_comm.join()
            thread_manual_control.join()

            print("Closing main.py..")
            return

        #print(q_ball.get())


main()

