import queue
import threading
import time

import cv2

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
    basket = bask.Basket("../thresh/thresh_basket_pink.txt")
    balls = ball.Balls("../thresh/thresh_ball.txt")
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
        distance = float(input("Input basket distance. \n(float)> "))
        print("Driving to throwing distance (based on basket size) ...")
        drive_to_distance(robot, distance)
        print("In throwing distance (based on basket size).", end="\n\n")

        # thrower_speed = int(input("Input thrower speed. \n(int)> "))
        # print("Throwing...")
        # throw_ball(robot, thrower_speed)
        # print("Ball has been thrown!", end="\n\n")
        #
        # save_values = input("Do you want to save these values (basket_size=" + str(distance) + "; thrower_speed=" + str(thrower_speed) + ")? \n(y/n)> ")
        # if save_values == "y":
        #     save_to_file("thrower_data.txt", distance, thrower_speed)
        recalibrate = input("Do you wish to recalibrate thrower (ENTER for yes, \"n\" for no)? ")


"""
    ABIFUNKTSIOONID
"""


def drive_to_distance(robot, distance):
    everything_ok = False
    while not everything_ok:

        print("Centering basket...")
        while not is_basket_centered(robot):
            robot.camera.find_objects()
            rotation = rot_movement(robot)
            print("Rotating. Basket x= " + str(robot.basket.get_x()) + ".")
            robot.mainboard.send_motors([0, 0, rotation])
        print("Basket centered.")

        robot.mainboard.send_motors([0, 0, 0])

        print("Driving to throwing distance...")
        while not is_distance_ok(robot, distance):
            robot.camera.find_objects()
            robot.mainboard.send_motors([0, y_movement(robot, distance), 0])
            print("Driving to basket. Basket x=" + str(robot.basket.get_x()) + "; Basket distance=" + str(
                robot.camera.get_distance_to_basket()))
            if abs(robot.basket.get_x() - robot.img_center) > robot.hysteresis_basket * 15:
                break
        print("Stopping.")

        everything_ok = is_basket_really_centered(robot) and is_distance_really_ok(robot, distance)

    print("In throwing distance (distance=" + str(robot.camera.get_distance_to_basket()) + ") and ready to throw.")
    robot.mainboard.send_motors([0, 0, 0])
    return


def throw_ball(robot, thrower_speed):
    robot.set_calibration_mode(True)
    robot.set_calibration_speed(thrower_speed)

    ball_thrown = False
    while not ball_thrown:

        robot.previous_time = robot.time
        robot.time = time.time()
        robot.period = robot.time - robot.previous_time

        robot.img_center, robot.img_height = robot.camera.find_objects()

        robot.ball_x = robot.balls.get_x()
        robot.previous_ball_y = robot.ball_y
        robot.ball_y = robot.balls.get_y()
        robot.basket_x = robot.basket.get_x()

        if robot.throwing_state >= 1:
            robot.throwing_logic()
            if robot.throwing_state == 0:
                ball_thrown = True
            continue

        # Check if the basket is too close
        if robot.basket.get_diameter() > robot.basket_max_d:
            robot.rotate_180_degrees()

        # NB! Not sure if necessary..
        # print("Counter:", counter)
        if robot.counter >= 15:
            robot.counter = 0
            robot.find_ball = not robot.find_ball

        # If we haven't found, rotated and moved to the ball:
        # print("Is ball found?")
        if robot.find_ball:
            if robot.ball_x > robot.img_center:
                robot.sign = 1
            else:
                robot.sign = -1
            robot.rotate_move_to_ball()

        else:
            robot.rotate_to_basket()

    robot.throwing_state = 0
    robot.mainboard.send_thrower(100)


    robot.mainboard.send_motors([0, 0, 0])
    robot.set_calibration_mode(False)

    return


def save_to_file(filename, distance, thrower_speed):
    f = open(filename, "a")
    f.write(str(distance) + "," + str(thrower_speed))
    f.write("\n")
    f.flush()
    f.close()


"""
    ABI-ABIFUNKTSIOONID
"""


def y_movement(robot, distance):
    error_speed = calculate_error(distance, robot.camera.get_distance_to_basket())
    if abs(error_speed) > 1:
        error_speed = 1 * (error_speed / abs(error_speed))
    if error_speed < 0:
        return -0.05 + error_speed
    return 0.05 + error_speed


def x_movement(robot):
    error_speed = (calculate_error(robot.basket.get_x(), robot.img_center) / 320) / 2
    if error_speed < 0:
        return -0.03 + error_speed
    return 0.03 + error_speed


def rot_movement(robot):
    error_speed = (calculate_error(robot.basket.get_x(), robot.img_center) / 320) / 6
    if error_speed < 0:
        return -0.02 + error_speed
    return 0.02 + error_speed


def calculate_error(value1, value2):
    error = value1 - value2
    return error


def is_distance_ok(robot, distance):
    return abs(robot.camera.get_distance_to_basket() - distance) < 0.02


def is_distance_really_ok(robot, requested_size):
    counter_not_ok = 0
    counter_ok = 0
    while counter_not_ok < 5 and counter_ok < 20:
        robot.camera.find_objects()
        if is_distance_ok(robot, requested_size):
            counter_ok += 1
        else:
            counter_not_ok += 1
    return counter_not_ok < 5


def is_basket_centered(robot, custom_hysterisis=None):
    if custom_hysterisis is None:
        return abs(robot.basket.get_x() - robot.img_center) < robot.hysteresis_basket
    return abs(robot.basket.get_x() - robot.img_center) < custom_hysterisis


def is_basket_really_centered(robot):
    counter_not_ok = 0
    counter_ok = 0
    while counter_not_ok < 5 and counter_ok < 20:
        robot.camera.find_objects()
        if is_basket_centered(robot):
            counter_ok += 1
        else:
            counter_not_ok += 1
    return counter_not_ok < 5


main()
