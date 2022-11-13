import serial
from serial.tools import list_ports
from math import *
import queue
import time
import motors
import cv2
import numpy as np


class Robot:
    def __init__(self, mainboard, camera, autonomy, stop_flag, balls, basket):
        self.name = "Placeholder.robot"
        self.mainboard = mainboard
        self.motors = motors.Motors()
        self.camera = camera
        self.autonomy = autonomy
        self.stop_flag = stop_flag
        self.balls = balls
        self.basket = basket
        self.thrower_speed = 100

        # Variables related to balls and basket
        self.ball_x = 320
        self.ball_y = 0
        self.basket_x = 320

        # Variables related to camera
        self.img_center = 320
        self.img_height = 480
        self.ball_y_stop = 325

        # All movement speeds
        self.rotation_speed = 0.03
        self.rotation_speed_basket = 0.2
        self.movement_speed = 0.07
        # Single wheel speed
        self.wheel_speed = 3

        # Proportional controller values
        self.gain_ball = 0.45
        self.gain_basket = 30
        self.gain_movement = 0.8
        self.hysteresis = 7
        self.hysteresis_basket = 7
        self.error_movement = [0, 0, 0, 0, 0, 0]
        self.size = 0
        self.size_average = [0, 0, 0, 0, 0, 0]

        # Variables related to other flow logic
        self.find_ball = True
        self.rotate_counter = 0
        self.rotate = True
        self.counter = 0
        self.sign = 1
        # A variable so we don't print stuff 60x per second..
        printer_counter = 0

    def autopilot(self):
        print("Starting autopilot.")

        # Main loop
        while not self.stop_flag.is_set():

            # Fetch data about frame, balls and basket.

            self.img_center, self.img_height = self.camera.find_objects()

            self.ball_x = self.balls.get_x()
            self.ball_y = self.balls.get_y()
            self.basket_x = self.basket.get_x()
            # print("BASKET:", basket_x)

            # Find the vertical stop value independent of frame height.
            # self.ball_y_stop = 0.73 * self.img_height
            #print("Basket diameter:", self.basket.get_diameter())
            #print("Closest ball: ", (self.ball_x, self.ball_y))
            # If the stop flag has not been set, the robot will stay operational.

            # If the robot is autonomous, the robot will execute its game logic.
            if self.autonomy.is_set():

                # NB! Not sure if necessary..
                # print("Counter:", counter)
                if self.counter >= 15:
                    self.counter = 0
                    self.find_ball = not self.find_ball

                # print("BALL: (" + str(ball_x) + "; " + str(ball_y) + ")")

                # If we haven't found, rotated and moved to the ball:
                # print("Is ball found?")
                if self.find_ball:
                    # print("Ball is not found.")
                    if self.ball_x > self.img_center:
                        self.sign = 1
                    else:
                        self.sign = -1

                    # Do until ball is in front of us (ball_y > ...)
                    # print("Is ball close enough to robot?")

                    # print("BALL")
                    self.rotate_move_to_ball()

                # If we have found the ball and want to rotate to the basket:
                else:
                    # print("BASKET")
                    self.rotate_to_basket()

    def rotate_move_to_ball(self):
        # print(self.ball_y, self.ball_y_stop)
        if self.ball_x == 0:
            if self.rotate:
                # print("Searching for ball")
                self.motors = [0, 0, -0.5]
            else:
                self.motors = [0, 0, 0]

            if self.rotate_counter > 5:
                self.rotate = not self.rotate
                self.rotate_counter = 0
            else:
                self.rotate_counter += 1

        elif self.ball_y > self.ball_y_stop:
            # print("Ball is close enough to robot!")
            error = abs((self.ball_x - self.img_center) / self.img_center)

            if abs(self.img_center - self.ball_x) < self.hysteresis:
                # print("Y-coord is good and ball is centered.")
                self.motors = [0, 0, 0]
                self.counter += 1
                # print(self.counter)
                # find_ball = False
            else:
                #print("Y-coord is good and ball is not centered.")
                self.counter = 0
                self.motors = [0, 0, self.sign * self.rotation_speed + self.sign * self.gain_ball * error]

        # If the ball is NOT in front of us.
        else:
            # Field of view ~ 90 degrees
            # 45 is in the middle
            # So we can convert pixels to degrees:
            # ball_degrees = ball_position compared to image center divided by
            # a constant, which is the ration between pixels and degrees
            # print("Ball is not close enough to robot!")

            # print("Set __motors to drive to ball.")
            self.error_movement.append(abs(self.ball_y_stop - self.ball_y) / self.ball_y_stop)
            self.error_movement = self.error_movement[1:]

            ball_degrees = (self.ball_x - self.img_center) / 7.11
            ball_degrees_rad = radians(ball_degrees)
            # Define y_speed as constant, because we always need to move forward
            # Then based on the angle we can calculate the x_speed
            y_speed = self.movement_speed + self.gain_movement * sum(self.error_movement) \
                      / len(self.error_movement)
            x_speed = tan(ball_degrees_rad) * y_speed

            self.motors = [-x_speed, -y_speed, 0]
        # Send motor speeds to rotate to the closest ball
        # print("Move robot!")
        self.mainboard.send_motors(self.motors)

    def rotate_to_basket(self):
        # Analytically calculate the thrower speed
        x = self.basket.get_diameter()
        # __thrower_speed = int(-0.0049 * (x ** 3) + 0.6311 * (x ** 2) - 26.674 * x + 543.7782)

        if x > 120:
            thrower_speed = 145
        elif x > 88:
            thrower_speed = 150
        elif x > 0:
            thrower_speed = int(375.4122332161802 * x ** (-0.1723588087308))
        else:
            thrower_speed = 0

        # if printer_counter > 60:
        #     print("d:", self.basket.get_diameter())
        #     print("Thrower:", __thrower_speed)
        #     print("BASKET:", basket_x)
        #     printer_counter = 0
        # else:
        #     printer_counter += 1
        # print("Finding basket...")
        if self.basket_x > self.img_center:
            self.sign = 1
        else:
            self.sign = -1

        # Calculate the error for P-controller
        # error = abs((self.basket_x - self.img_center) / self.img_center)

        # If the basket is in the center of our view
        # print("Is basket centered?")
        if abs(self.img_center - self.basket_x) < self.hysteresis:
            # print("Basket centered!")
            # If the ball is NOT in the center of our view then find ball again
            #print("Is ball still in center?")
            if abs(self.img_center - self.ball_x) > 2 * self.hysteresis or self.ball_x == 0:
                print("Ball not in center!")
                self.find_ball = True
                return
            # __motors = [0, 0, 0]
            # print("Ball in center!")

            # print("Is ball centered for enough time?")
            if self.counter > 7:
                # print("Ball centered for enough time!")

                # Send motor and thrower speeds to mainboard
                self.mainboard.send_motors([0, 0, 0])

                # Thrower logic without using a timeout (time.sleep()), which caused serial problems.
                def throwing_logic():
                    # Epoch time in float seconds
                    x = self.basket.get_diameter()

                    #__thrower_speed = 269.5 + (-2.89 * x) + (0.0209 * x ** 2)

                    #__thrower_speed = 198

                    print("Thrower speed:", thrower_speed)
                    print("Basket diameter:", x)
                    start_time = time.time()

                    print("Throwing!")
                    # Throwing..
                    # __thrower_speed += 20

                    self.mainboard.send_thrower(thrower_speed)
                    time.sleep(0.7)
                    self.mainboard.send_motors([0, -0.6, 0])
                    self.mainboard.send_thrower(thrower_speed)
                    time.sleep(0.02)

                    time.sleep(1)

                    # while current_time - start_time < 1.5:
                    #     self.mainboard.send_motors([0, -0.6, 0])
                    #     current_time = time.time()
                    #     time.sleep(0.01)
                    self.mainboard.thrower_speed = 100

                    """
                    while True:
                        current_time = time.time()
    
                        if current_time >= start_time + 2.7:
                            self.mainboard.send_thrower(125)
                            break
                        elif current_time >= start_time + 0.7 and send:
                            self.mainboard.send_motors_raw([0.6, -0.6, 0.6])
                            send = False
                        elif current_time >= start_time + 1.0 and send_second_thrower:
                            self.mainboard.send_thrower(__thrower_speed)
                            send_second_thrower = False
                            """

                throwing_logic()

                self.counter = 0
                self.find_ball = True

            else:
                # print("Ball is not centered for enough time. Increasing counter.")
                self.counter += 1
            # find_ball = True

        # If basket not in center
        else:
            print("Basket is not centered.")

            def estimate_distance(size):
                if size != 0:
                    return 59.97575225 * size ** (-1.398997)
                return 0

            def average_size():
                return sum(self.size_average) / len(self.size_average)

            # self.mainboard.send_motors_raw([0, wheel_speed * sign + sign * gain_basket * error, 0])

            self.counter = 0

            # If we have not yet seen any ball, we have to initialize the value
            if not self.size == 0:
                previous_size = self.size
                self.size = self.balls.get_size()
                self.size_average.append(self.size)
                self.size_average = self.size_average[1:]
            else:
                self.size = previous_size = self.balls.get_size()
                # Fill the list
                for i in range(len(self.size_average)):
                    self.size_average.append(self.size)
                    self.size_average = self.size_average[1:]

            self.rotation_speed_basket = 0.004
            rotation_speed_constant = 0.05

            print("Ball size:", self.size, "; average size:", average_size(), self.size_average)

            gain_temp = 0.3
            error = abs((self.basket_x - self.img_center) / self.img_center)

            if error >= 0.1:
                if abs(self.ball_x - self.img_center) > 6 * self.hysteresis:
                    print("Ball not in center!")
                    self.find_ball = True
                    return
                self.rotation_speed_basket = self.rotation_speed_basket + gain_temp * error / 2

                size_error = (self.size - previous_size) / average_size() * 50
                max_error = 0.8
                if size_error > max_error:
                    size_error = max_error
                elif size_error < -max_error:
                    size_error = -max_error
                print("Size error:", size_error)
                translational_speed = estimate_distance(average_size()) * self.rotation_speed_basket * 18.9 + size_error

                # print("X_speed:", translational_speed, "Rot:", rotation_speed)
                print("Rotating around the ball.", self.rotation_speed_basket, translational_speed, error)
                motors = [translational_speed * self.sign, 0, self.rotation_speed_basket * self.sign]
                self.mainboard.send_motors(motors)

            else:
                gain_wheel = 55
                error = abs((self.basket_x - self.img_center) / self.img_center)

                wheel_speed_temp = 5
                print(wheel_speed_temp * gain_wheel * error, error)

                if self.basket_x - self.img_center > self.hysteresis_basket:
                    self.mainboard.send_motors_raw([0, wheel_speed_temp + gain_wheel * error, 0])
                elif self.basket_x - self.img_center < self.hysteresis_basket:
                    self.mainboard.send_motors_raw([0, -wheel_speed_temp - gain_wheel * error, 0])

            # gain_temp = 75
            # error = abs((self.basket_x - self.img_center) / self.img_center)
            #
            # wheel_speed_temp = 6
            # print(wheel_speed_temp * gain_temp * error, error)
            #
            # if self.basket_x - self.img_center > self.hysteresis_basket:
            #     self.mainboard.send_motors_raw([0, wheel_speed_temp + gain_temp * error, 0])
            # elif self.basket_x - self.img_center < self.hysteresis_basket:
            #     self.mainboard.send_motors_raw([0, -wheel_speed_temp - gain_temp * error, 0])
