import serial
from serial.tools import list_ports
from math import *
import queue
import time

import cv2
import numpy as np


class Robot:
    def __init__(self, mainboard, camera, autonomy, stop_flag, balls, basket):
        self.name = "Placeholder.robot"
        self.mainboard = mainboard
        self.motors = Motors()
        self.camera = camera
        self.autonomy = autonomy
        self.stop_flag = stop_flag
        self.balls = balls
        self.basket = basket

    def autopilot(self):

        robot = self
        # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
        # the robot won't move.
        hysteresis = 7
        print("Starting autopilot.")
        # The center of our camera image
        _, frame = robot.camera.cap.read()
        frame_x, frame_y = robot.camera.find_image_dimensions(frame)
        #ball_y_stop = 0.80 * frame_y
        ball_y_stop = 320
        print("Ball y stop: ", ball_y_stop)
        img_center = frame_x / 2

        gain_ball = 0.4
        gain_basket = 30
        gain_movement = 0.01
        # When to stop moving (distance from the center)
        offset = 0.015

        # Default speed for rotation and movement
        rotation_speed = 0.03
        movement_speed = 0.6
        min_speed = 0.06
        # movement_speed = min_speed + 0.2
        # Additional variable for correcting the rotation.
        correction = 0

        error_movement = [0, 0, 0, 0, 0]
        # Single wheel speed
        wheel_speed = 3

        # Basket or ball
        find_ball = True
        counter = 0

        while not robot.stop_flag.is_set():

            _, frame = robot.camera.cap.read()
            balls = robot.camera.find_balls_xy(frame)
            basket_x, basket_diameter = robot.camera.find_basket(frame)

            #print("Balls:", balls, "len(balls) =", len(balls), "robot.stop_flag.is_set() = ", robot.stop_flag.is_set())

            if robot.autonomy.is_set():

                # Phase 1. Find balls.
                if len(balls) == 0:
                    print("Finding balls!")
                    robot.rotate(img_center, 0, 100, 0, rotation_speed, hysteresis, gain_ball)
                    continue
                print("Found balls.\n")

                # Phase 2. Drive to closest ball.
                closest_ball = balls[0]
                if closest_ball[1] < ball_y_stop:
                    print("Driving towards ball:", closest_ball)
                    robot.drive_to_ball(closest_ball, ball_y_stop, movement_speed, gain_movement, error_movement, img_center)
                    continue
                print("Close enough to ball.\n")

                ball_y = closest_ball[1]
                ball_x = closest_ball[0]



                """
                # Phase 3. Rotate around the ball into a throwing position.
                if not robot.stop_flag.is_set() and abs(basket_x - img_center) < hysteresis:
                    _, frame = robot.camera.cap.read()

                    balls = robot.camera.find_balls_xy(frame)

                    if len(balls) == 0:
                        continue

                    closest_ball = balls[0]
                    closest_ball_size = robot.camera.find_ball_size(frame, closest=True)
                    rotate_around_ball(closest_ball, closest_ball_size)

                    _, frame = robot.camera.cap.read();
                    balls = robot.camera.find_balls_xy(frame)
                    basket_x, diameter = robot.camera.find_basket(frame)

                    if len(balls) == 0:
                        continue

                    closest_ball = balls[0]
                """

                # Phase 4. Throwing ball.
                # Send motor and thrower speeds to mainboard

                # # Thrower logic without using a timeout (time.sleep()), which caused serial problems.
                # if not robot.stop_flag.is_set():
                #     print("Throwing!")
                #     threw_ball = robot.throwing_logic(basket_diameter)
                #     if threw_ball:
                #         robot.autonomy.clear()
                #         print("Ball thrown!\n")

    def rotate(self, img_center, ball_x, ball_y, ball_y_stop, rotation_speed, hysteresis, gain_ball):
        if ball_x > img_center:
            sign = 1
        else:
            sign = -1

            # Do until ball is in front of us (ball_y > ...)
        if ball_y > ball_y_stop:
            error = abs((ball_x - img_center) / img_center)

            if abs(img_center - ball_x) < hysteresis:
                motors = [0, 0, 0]
                # find_ball = False
            else:
                motors = [0, 0, sign * rotation_speed + sign * gain_ball * error]

            self.mainboard.send_motors(motors)

    def drive_to_ball(self, ball, ball_y_stop, movement_speed, gain_movement, error_movement, img_center):
        ball_x = ball[0]
        ball_y = ball[1]

        error_movement.append(abs(ball_y_stop - ball_y))
        error_movement = error_movement[1:]

        ball_degrees = (ball_x - img_center) / 7.11
        ball_degrees_rad = radians(ball_degrees)
        # Define y_speed as constant, because we always need to move forward
        # Then based on the angle we can calculate the x_speed
        y_speed = movement_speed * 1 * gain_movement * sum(error_movement) / len(error_movement)
        x_speed = tan(ball_degrees_rad) * y_speed

        motors = [-x_speed, -y_speed, 0]
        # Send motor speeds to rotate to the closest ball

        self.mainboard.send_motors(motors)

    def rotate_around_ball(self, ball, ball_size):
        return

    def throwing_logic(self, basket_diameter):

        thrower_speed = int(-0.0049 * (basket_diameter ** 3) + 0.6311 * (basket_diameter ** 2) - 26.674 * basket_diameter + 543.7782)

        # Epoch time in float seconds
        start_time = time.time()
        send = True
        send_second_thrower = True

        # Throwing..
        self.mainboard.send_thrower(thrower_speed)
        while not self.stop_flag.is_set():
            current_time = time.time()

            if current_time >= start_time + 2.7:
                self.mainboard.send_thrower(125)
                break
            elif current_time >= start_time + 0.7 and send:
                self.mainboard.send_motors([0, -0.6, 0])
                send = False
            elif current_time >= start_time + 1.0 and send_second_thrower:
                self.mainboard.send_thrower(thrower_speed)
                send_second_thrower = False
        return True


"""
    def autopilot(self):
        # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
        # the robot won't move.
        hysteresis = 7
        print("Starting autopilot.")
        # The center of our camera image
        ball_y_stop = 350

        gain_ball = 0.4
        gain_basket = 30
        gain_movement = 0.01
        # When to stop moving (distance from the center)
        offset = 0.015

        # Default speed for rotation and movement
        rotation_speed = 0.03
        movement_speed = 0.6
        min_speed = 0.06
        #movement_speed = min_speed + 0.2
        # Additional variable for correcting the rotation.
        correction = 0

        error_movement = [0, 0, 0, 0, 0]
        # Single wheel speed
        wheel_speed = 3

        # Basket or ball
        find_ball = True
        counter = 0

        # A variable so we don't print stuff 60x per second..
        printer_counter = 0

        while True:
            img_center, img_height = self.camera.find_objects()
            ball_x = self.balls.get_x()
            ball_y = self.balls.get_y()
            basket_x = self.basket.get_x()
            #print("BASKET:", basket_x)

            ball_y_stop = 0.73 * img_height

            if not self.stop_flag.is_set():
                #print("Auto:", self.autonomy.is_set())
                if self.autonomy.is_set():
                    # print("Counter:", counter)
                    if counter >= 15:
                        counter = 0
                        find_ball = not find_ball

                    # print("BALL: (" + str(ball_x) + "; " + str(ball_y) + ")")

                    # Controller logic:
                    # if ball is to our right, rotate right;
                    # if it's to our left, rotate left;
                    # if it's kind of in the middle, don't do anything (hysteresis)
                    print("Is ball found?")
                    if find_ball:
                        print("Ball is not found.")
                        if ball_x > img_center:
                            sign = 1
                        else:
                            sign = -1

                        # Do until ball is in front of us (ball_y > ...)
                        print("Is ball close enough to robot?")
                        if ball_y > ball_y_stop:
                            print("Ball is close enough to robot!")
                            error = abs((ball_x - img_center) / img_center)

                            if abs(img_center - ball_x) < hysteresis:
                                print("Y-coord is good and ball is centered.")
                                motors = [0, 0, 0]
                                counter += 1
                                print(counter)
                                #find_ball = False
                            else:
                                print("Y-coord is good and ball is not centered.")
                                counter = 0
                                motors = [0, 0, sign * rotation_speed + sign * gain_ball * error]
                        else:
                            # Field of view ~ 90 degrees
                            # 45 is in the middle
                            # So we can convert pixels to degrees:
                            # ball_degrees = ball_position compared to image center divided by
                            # a constant, which is the ration between pixels and degrees
                            print("Ball is not close enough to robot!")

                            print("Set motors to drive to ball.")
                            error_movement.append(abs(ball_y_stop - ball_y))
                            error_movement = error_movement[1:]

                            ball_degrees = (ball_x - img_center) / 7.11
                            ball_degrees_rad = radians(ball_degrees)
                            # Define y_speed as constant, because we always need to move forward
                            # Then based on the angle we can calculate the x_speed
                            y_speed = movement_speed * 1 * gain_movement * sum(error_movement)/len(error_movement)
                            x_speed = tan(ball_degrees_rad) * y_speed

                            motors = [-x_speed, -y_speed, 0]
                        # Send motor speeds to rotate to the closest ball
                        print("Move robot!")
                        self.mainboard.send_motors(motors)

                    # If not find_ball
                    else:
                        # Analytically calculate the thrower speed
                        x = self.basket.get_diameter()
                        thrower_speed = int(-0.0049 * (x ** 3) + 0.6311 * (x ** 2) - 26.674 * x + 543.7782)

                        # if printer_counter > 60:
                        #     print("d:", self.basket.get_diameter())
                        #     print("Thrower:", thrower_speed)
                        #     print("BASKET:", basket_x)
                        #     printer_counter = 0
                        # else:
                        #     printer_counter += 1
                        print("Finding basket...")
                        if basket_x > img_center:
                            sign = 1
                        else:
                            sign = -1

                        error = abs((basket_x - img_center) / img_center)

                        # If the basket is in the center of our view
                        print("Is basket centered?")
                        if abs(img_center - basket_x) < hysteresis:
                            print("Basket centered!")
                            # If the ball is NOT in the center of our view then find ball again
                            print("Is ball still in center?")
                            if abs(img_center - ball_x) > hysteresis or ball_x == 0:
                                print("Ball not in center!")
                                find_ball = True
                                continue
                            #motors = [0, 0, 0]
                            print("Ball in center!")

                            print("Is ball centered for enough time?")
                            if counter > 10:
                                print("Ball centered for enough time!")
                                # Send motor and thrower speeds to mainboard

                                # Thrower logic without using a timeout (time.sleep()), which caused serial problems.
                                def throwing_logic(thrower_speed):
                                    # Epoch time in float seconds
                                    start_time = time.time()
                                    send = True
                                    send_second_thrower = True

                                    print("Throwing!")
                                    # Throwing..
                                    self.mainboard.send_thrower(thrower_speed)
                                    while True:
                                        current_time = time.time()

                                        if current_time >= start_time + 2.7:
                                            self.mainboard.send_thrower(125)
                                            break
                                        elif current_time >= start_time + 0.7 and send:
                                            self.mainboard.send_motors([0, -0.6, 0])
                                            send = False
                                        elif current_time >= start_time + 1.0 and send_second_thrower:
                                            self.mainboard.send_thrower(thrower_speed)
                                            send_second_thrower = False

                                throwing_logic(thrower_speed)

                                counter = 0
                                find_ball = True
                            else:
                                print("Ball is not centered for enough time. Increasing counter.")
                                counter += 1
                            # find_ball = True

                        # If basket not in center
                        else:
                            print("Basket is not centered.")
                            def estimate_distance(size):
                                if size != 0:
                                    return 59.97575225 * size ** (-1.398997)
                                return 0
                            #self.mainboard.send_motors_raw([0, wheel_speed * sign + sign * gain_basket * error, 0])

                            counter = 0

                            rotation_speed = 0.2

                            size = self.balls.get_size()

                            translational_speed = estimate_distance(size) * rotation_speed * 12

                            #print("X_speed:", translational_speed, "Rot:", rotation_speed)
                            print("Rotating around the ball.")
                            motors = [translational_speed, 0, rotation_speed]
                            self.mainboard.send_motors(motors)

                            #self.rotate_around_ball()
                            find_ball = True
                            #send_to_mainboard([0, 10, 0])
                            #motors = [sign * movement_speed, 0, sign * rotation_speed]
                            #if abs(img_center - ball_x) > 10:
                            #    self.rotateManual();
                            #self.omniDirectional(self.balls.get_x(), self.balls.get_y())

                            #if basket_x - img_center > 20:
                            #    self.mainboard.send_motors_raw([10, -20, 0])
                            #elif basket_x - img_center < 20:
                            #    self.mainboard.send_motors_raw([-10, 20, 0])

"""


class Mainboard:
    # Class to communicate with the mainboard.
    # The only class to have direct access to it.

    def __init__(self, autonomy, stop_flag):
        self.name = "Placeholder.mainboard"
        # Initialize the serial port
        self.ser = Mainboard.get_mainboard_serial_port()
        # Queues to hold the information going to and coming from the mainboard
        self.__to_mainboard = queue.Queue(1)
        self.__from_mainboard = queue.Queue()
        self.stop_flag = stop_flag

        # Queue timeout
        self.__timeout = 0.01

        # Field and robot ID
        self.field = "B"
        self.id = "A"

        self.autonomy = autonomy

    @staticmethod
    # Scan for mainboard serial ports
    def get_mainboard_serial_port():
        ports = list_ports.comports()
        for port in ports:
            try:
                ser = serial.Serial(port.device, 9600, timeout=0.02)
                return ser
            except:
                continue
        raise Exception("Could not find suitable or any USB ports.")

    # COMMUNICATION FROM OTHER CLASSES
    # Generic method to send any string to mainboard
    def send(self, message):
        self.send_to_mainboard(message)

    # Method to send motor speeds to mainboard
    def send_motors_raw(self, motors):
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        print(message)
        self.send_to_mainboard(message)

    # Method to send motor speeds to mainboard
    def send_motors(self, motors):
        motors = Motors.get_motor_speeds(motors[0], motors[1], motors[2])
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        # self.__to_mainboard.put(message, timeout=self.__timeout)
        self.send_to_mainboard(message)

    def send_stop(self):
        message = "sd:0:0:0:0\n".encode("'utf-8")
        self.send_to_mainboard(message)

    def send_thrower(self, thrower_speed):
        message = ("d:" + str(round(thrower_speed)) + "\n").encode("'utf-8")
        self.send_to_mainboard(message)

    # ACTUAL SERIAL COMMUNICATION
    # Method to communicate with the mainboard
    # This will both write and receive messages 60 times per second
    def send_to_mainboard(self, message):
        self.ser.write(message)
        time.sleep(0.009)

        response = self.poll_mainboard()
        # Referee commands, responding in real-time
        if "ref" in response:
            # print("REFEREE COMMAND!")
            self.ref_response(response)
            # Print error message for debugging
        elif "buffer empty" in response:
            a = None

    # This method in an endless loop??
    def poll_mainboard(self):
        if self.ser.in_waiting:
            line = self.ser.read(20).decode()

            # Dump the message to a queue of messages and also return it for immediate use.
            # self.__from_mainboard.put(line)
            return line

        return "buffer empty"

        # Return error message for debugging
        # return "Input buffer empty!"

    def ref_response(self, response):
        # Filter the response
        start_index = response.find(":") + 1
        end_index = response.find(">")
        ref_command = response[start_index:end_index]

        robot_id = ref_command[1:3]
        data = ref_command[3:12]

        print("ID:", robot_id)
        # If the field matches:
        if robot_id[0] == self.field:
            # Send the acknowledge response first if the command is specific to our robot
            if robot_id[1] == self.id:
                acknowledge = ("rf:a" + self.field + self.id + "ACK-----\n").encode("utf-8")
                self.ser.write(acknowledge)
                time.sleep(0.01)
            # If the command is to stop
            if "STOP" in data:
                # If the robot id matches ours OR "X" (all robots)
                if robot_id[1] == "X" or robot_id[1] == self.id:
                    # Send the stop signal
                    # self.ser.write("sd:0:0:0:0\n".encode("utf-8"))
                    self.autonomy.clear()
            # If the command is to start the game
            elif "START" in data:
                # If the robot id matches ours OR "X" (all robots)
                if robot_id[1] == "X" or robot_id[1] == self.id:
                    # Start the game
                    self.autonomy.set()


# Omniwheel motion logic
class Motors:
    # wheelSpeedToMainboardUnits = gearboxReductionRatio * encoderEdgesPerMotorRevolution /
    # (2 * PI * wheelRadius * pidControlFrequency)
    wheel_speed_to_mainboard_units = 18.75 * 64 / (2 * pi * 0.035 * 60)

    @staticmethod
    def robot_speed(robot_speed_x, robot_speed_y):
        return sqrt(robot_speed_x ** 2 + robot_speed_y ** 2)

    @staticmethod
    def robot_direction_angle(robot_speed_x, robot_speed_y):
        return atan2(robot_speed_y, robot_speed_x)

    @staticmethod
    def wheel_linear_velocity(robot_speed, robot_direction_angle, wheel_angle):
        # print("Robot_speed:", robot_speed, "; angle:", robot_direction_angle , "; cos:", cos(robot_direction_angle - wheel_angle))
        return robot_speed * cos(robot_direction_angle - radians(wheel_angle))

    @staticmethod
    def wheel_angular_speed_mainboard_units(wheel_linear_velocity, wheel_speed_to_mainboard_units):
        # print("Wheel linear velocity:", wheel_linear_velocity)
        return wheel_linear_velocity * wheel_speed_to_mainboard_units

    @staticmethod
    def get_motor_speeds(robot_speed_x, robot_speed_y, rotation):
        rbt_spd = Motors.robot_speed(robot_speed_x, robot_speed_y)
        dir_ang = Motors.robot_direction_angle(robot_speed_x, robot_speed_y)
        # print("Speed:", rbt_spd, "; angle:", degrees(dir_ang))

        rot_constant = Motors.wheel_speed_to_mainboard_units
        rot = rotation * rot_constant

        motors = [0, 0, 0]
        motors[0] = Motors.wheel_angular_speed_mainboard_units(Motors.wheel_linear_velocity(rbt_spd, dir_ang, 120),
                                                               Motors.wheel_speed_to_mainboard_units) + rot
        motors[1] = Motors.wheel_angular_speed_mainboard_units(Motors.wheel_linear_velocity(rbt_spd, dir_ang, 0),
                                                               Motors.wheel_speed_to_mainboard_units) + rot
        motors[2] = Motors.wheel_angular_speed_mainboard_units(Motors.wheel_linear_velocity(rbt_spd, dir_ang, 240),
                                                               Motors.wheel_speed_to_mainboard_units) + rot
        return motors
