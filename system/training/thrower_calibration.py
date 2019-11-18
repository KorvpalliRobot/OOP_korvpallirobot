import queue
import threading

import system.balls as ball
import system.basket as bask
import system.camera as cam
import system.mainboard as mainb
import system.robot as r
import system.remote_control as remote_control

def main():
    # DATA
    # Holds the information about game logic state (game or manual)
    autonomy = threading.Event()
    autonomy.set()
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
    mainboard = mainb.Mainboard(autonomy, stop_flag)
    robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)
    # For testing only thrower using remote control
    # robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket, q_thrower_speed)

    # Manual control
    thread_manual_control = threading.Thread(name="manual", target=remote_control.gamepad,
                                             args=(mainboard, autonomy, stop_flag, q_thrower_speed), daemon=True)
    # Mainboard communication
    thread_mainboard = threading.Thread(name="mainboard", target=mainboard.send_to_mainboard, daemon=True)

    thread_manual_control.start()
    thread_mainboard.start()

    recalibrate = ""
    while recalibrate == "":
        distance = int(input("Input basket size. \n> "))

        drive_backwards(distance, robot)

        input("Press enter when ready to throw.\n")
        print("Throwing...")
        throw_ball(robot)
        print("Ball has been thrown!", end="\n\n")

        recalibrate = input("Do you wish to recalibrate thrower (ENTER for yes, \"n\" for no? \n> ")


# The main loop for our program, use to display values etc
"""
    ABIFUNKTSIOONID
"""


def drive_backwards(requested_size, robot):
    everything_ok = False
    while not everything_ok:

        while not is_basket_really_centered(robot):
            error = calculate_error(robot.basket.get_x(), robot.img_center)
            robot.mainboard.send_motors([0, 0, 0.01*error])

        while is_distance_really_ok(robot, requested_size):
            error = calculate_error(robot.basket.get_diameter(), requested_size)
            robot.mainboard.send_motors([0, 0.01*error, 0])
            if not is_basket_centered(robot):
                break

        everything_ok = is_basket_centered(robot) and is_distance_ok(robot, requested_size)

    return


def throw_ball(robot):

    return



"""
    ABI-ABIFUNKTSIOONID
"""


def calculate_error(value1, value2):
    error = value1 - value2
    if abs(error) > 100:
        error = error / abs(error) * 100
    return error


def is_distance_ok(robot, requested_size):
    return robot.basket.get_diameter == requested_size


def is_distance_really_ok(robot, requested_size):
    counter_not_ok = 0
    counter_ok = 0
    while counter_not_ok < 5 and counter_ok < 20:
        if is_distance_ok(robot, requested_size):
            counter_ok += 1
        else:
            counter_not_ok += 1
    return counter_not_ok < 5


def is_basket_centered(robot):
    return abs(robot.basket.get_x() - robot.img_center) > robot.hysteresis_basket


def is_basket_really_centered(robot):
    counter_not_ok = 0
    counter_ok = 0
    while counter_not_ok < 5 and counter_ok < 20:
        if is_basket_centered(robot):
            counter_ok += 1
        else:
            counter_not_ok += 1
    return counter_not_ok < 5


main()
