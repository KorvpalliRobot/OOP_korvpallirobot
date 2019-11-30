# import robot_thrower_calibration as r

import queue
import threading

import balls as ball
import basket as bask
import camera as cam
import mainboard as mainb
import remote_control
import robot as r


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
    basket = bask.Basket("thresh/thresh_basket_blue.txt")
    balls = ball.Balls("thresh/thresh_ball.txt")
    camera_thread = cam.ImageCapRS2(stop_flag)
    camera = cam.Camera(basket, balls, camera_thread, stop_flag)
    mainboard = mainb.Mainboard(autonomy, stop_flag)
    # For testing only thrower using remote control
    # robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket, q_thrower_speed)
    robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)

    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(mainboard, autonomy, stop_flag, q_thrower_speed), daemon=True)
    # Mainboard communication
    thread_mainboard = threading.Thread(name="mainboard", target=mainboard.send_to_mainboard, daemon=True)

    thread_manual_control.start()
    thread_mainboard.start()

    # The main loop for our program, use to display values etc
    robot.autopilot()


main()
