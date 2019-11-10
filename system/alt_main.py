#import robot_thrower_calibration as r
import time

import robot as r
import camera as cam
import basket as bask
import balls as ball
import threading
import remote_control
import queue


def main():
    # DATA
    # Holds the information about game logic state (game or manual)
    autonomy = threading.Event()
    autonomy.clear()
    # Stop signal for all threads
    stop_flag = threading.Event()
    stop_flag.clear()

    # For remotely changing thrower speed (works with robot_thrower_calibration as r)
    q_thrower_speed = queue.Queue()

    # OBJECTS
    basket = bask.Basket("thresh/thresh_basket_pink.txt")
    balls = ball.Balls("thresh/thresh_ball.txt")
    camera_thread = cam.ImageCapRS2(stop_flag)
    camera = cam.Camera(basket, balls, camera_thread, stop_flag)
    mainboard = r.Mainboard(autonomy, stop_flag)
    # For testing only thrower using remote control
    #robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket, q_thrower_speed)
    robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)

    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(mainboard, autonomy, stop_flag, q_thrower_speed), daemon=True)
    thread_manual_control.start()

    # The main loop for our program, use to display values etc
    robot.autopilot()


main()
