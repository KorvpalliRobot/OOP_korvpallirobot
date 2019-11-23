import time
from math import *

start_time = 0


class Robot:
    def __init__(self, mainboard, camera, autonomy, stop_flag, balls, basket):
        self.name = "Placeholder.robot"
        self.mainboard = mainboard
        self.motors = [0, 0, 0]
        self.camera = camera
        self.autonomy = autonomy
        self.stop_flag = stop_flag
        self.balls = balls
        self.basket = basket
        self.thrower_speed = 100
        self.throwing_state = 0
        self.time = time.time()
        self.previous_time = self.time
        self.period = 0

        # Variables for thrower calibration mode
        self.calibration_mode = False
        self.calibration_speed = None

        # Variables related to balls and basket
        self.ball_x = 320
        self.ball_y = 0
        self.previous_ball_y = 0
        self.basket_x = 320
        self.basket_max_d = 200

        # Variables related to camera
        self.img_center = 320
        self.img_height = 480
        self.ball_y_stop = 320
        # All movement speeds
        self.rotation_speed = 0.03
        self.rotation_speed_basket = 0.03
        self.movement_speed = 0.15
        # Single wheel speed
        self.wheel_speed = 3

        # Proportional controller values
        self.gain_ball = 0.45
        self.gain_basket = 30
        self.gain_movement = 1.7
        self.hysteresis = 7
        self.hysteresis_basket = 7
        self.error_movement = [0, 0, 0, 0, 0, 0]
        self.size = 0
        self.size_average = [0, 0, 0, 0]
        self.size_error = 0.01

        # PID controller values
        self.ball_integral = 0
        self.ball_derivative = 0

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

            # Find the time between two iterations
            self.previous_time = self.time
            self.time = time.time()
            self.period = self.time - self.previous_time

            self.img_center, self.img_height = self.camera.find_objects()

            self.ball_x = self.balls.get_x()
            self.previous_ball_y = self.ball_y
            self.ball_y = self.balls.get_y()
            self.basket_x = self.basket.get_x()
            # print("BASKET:", basket_x)

            # Find the vertical stop value independent of frame height.
            # self.ball_y_stop = 0.73 * self.img_height
            # print("Closest ball: ", (self.ball_x, self.ball_y))
            # If the stop flag has not been set, the robot will stay operational.

            # If the robot is autonomous, the robot will execute its game logic.
            # print("Basket diameter:", self.basket.get_diameter())
            if self.autonomy.is_set():

                if self.throwing_state >= 1:
                    self.throwing_logic()
                    continue

                # Check if the basket is too close
                if self.basket.get_diameter() > self.basket_max_d:
                    self.rotate_180_degrees()

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

            else:
                self.throwing_state = 0
                self.mainboard.send_thrower(100)

    def rotate_move_to_ball(self):
        # print(self.ball_y, self.ball_y_stop)
        if self.ball_x == 0:
            if self.rotate:
                # print("Searching for ball")
                self.motors = [0, 0, -0.5]
            else:
                self.motors = [0, 0, 0]

            if self.rotate_counter > 20:
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
                # print("Y-coord is good and ball is not centered.")
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

            # Use a function to make the error non-linear
            error_movement_avg = sum(self.error_movement) / len(self.error_movement)
            error_movement_avg = error_movement_avg ** 2

            ball_degrees = (self.ball_x - self.img_center) / 7.11
            ball_degrees_rad = radians(ball_degrees)
            # Define y_speed as constant, because we always need to move forward
            # Then based on the angle we can calculate the x_speed
            y_speed = self.movement_speed + self.gain_movement * error_movement_avg
            x_speed = tan(ball_degrees_rad) * y_speed

            self.motors = [-x_speed, -y_speed, 0]
        # Send motor speeds to rotate to the closest ball
        # print("Move robot!")
        self.mainboard.send_motors(self.motors)

    def rotate_to_basket(self):
        # Analytically calculate the thrower speed
        x = self.basket.get_diameter()
        # __thrower_speed = int(-0.0049 * (x ** 3) + 0.6311 * (x ** 2) - 26.674 * x + 543.7782)

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
            print("Basket centered!")
            # If the ball is NOT in the center of our view then find ball again
            # print("Is ball still in center?")
            ball_error = self.img_center - self.ball_x
            if abs(ball_error) > 2 * self.hysteresis or self.ball_x == 0:
                print("Ball not in center!")
                self.find_ball = True
                return
            # __motors = [0, 0, 0]
            print("Ball in center!")

            # print("Is ball centered for enough time?")
            if self.counter > 7:
                # print("Ball centered for enough time!")

                # Send motor and thrower speeds to mainboard
                self.mainboard.send_motors([0, 0, 0])

                # Thrower logic without using a timeout (time.sleep()), which caused serial problems.
                if self.throwing_state == 0:
                    global start_time
                    start_time = time.time()
                    self.throwing_state = 1

            else:
                # print("Ball is not centered for enough time. Increasing counter.")
                self.counter += 1
            # find_ball = True

        # If basket not in center
        else:
            # print("Basket is not centered.")

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

            gain_temp = 0.4
            error = abs((self.basket_x - self.img_center) / self.img_center)
            error **= 2

            # if error >= 0.1:
            if abs(self.ball_x - self.img_center) > 8 * self.hysteresis:
                print("Ball not in center!")
                self.find_ball = True
                self.ball_derivative = 0
                self.ball_integral = 0
                return
            self.rotation_speed_basket = self.rotation_speed_basket + gain_temp * error / 2

            # size_error = (self.size - previous_size) / average_size() * 50

            size_error = (self.ball_y - self.previous_ball_y) / 3
            print("Size error:", size_error)

            self.ball_integral += self.period * size_error
            self.ball_derivative = size_error / self.period

            k_p = 0.04
            k_i = 0.03
            k_d = 0.02

            # max_error = 1
            # if size_error > max_error:
            #     size_error = max_error
            # elif size_error < -max_error:
            #     size_error = -max_error

            translational_speed = estimate_distance(average_size()) * self.rotation_speed_basket * 18.5 \
                                  + k_p * size_error + k_i * self.ball_integral + k_d * self.ball_derivative

            # print("X_speed:", translational_speed, "Rot:", rotation_speed)
            print("Rotating around the ball.", self.rotation_speed_basket, translational_speed, error)
            motors = [translational_speed * self.sign, 0, self.rotation_speed_basket * self.sign]
            self.mainboard.send_motors(motors)

            # else:
            #     gain_wheel = 55
            #     error = abs((self.basket_x - self.img_center) / self.img_center)
            #
            #     wheel_speed_temp = 5
            #     print(wheel_speed_temp * gain_wheel * error, error)
            #
            #     if self.basket_x - self.img_center > self.hysteresis_basket:
            #         self.mainboard.send_motors_raw([0, wheel_speed_temp + gain_wheel * error, 0])
            #     elif self.basket_x - self.img_center < self.hysteresis_basket:
            #         self.mainboard.send_motors_raw([0, -wheel_speed_temp - gain_wheel * error, 0])

    def rotate_180_degrees(self):
        self.mainboard.send_motors([0, 0, 1.3])
        time.sleep(0.5)
        self.mainboard.send_motors([0, 0, 0])

    def throwing_logic(self):
        # Epoch time in float seconds
        x = self.basket.get_diameter()

        x -= 1

        # if x > 120:
        #     self.thrower_speed = 145
        # elif x > 88:
        #     self.thrower_speed = 150
        # elif x > 0:
        #     self.thrower_speed = int(375.4122332161802 * x ** (-0.1723588087308))
        # else:
        #     self.thrower_speed = 0

        # self.thrower_speed = int(278 - 2.46 * x + 0.0138 * x ** 2)

        #self.thrower_speed = 238 - (1.48 * x) + (6.91/1000 * x ** 2)
        # #round(450 + (-12.4 * x) + (0.207 * x ** 2) + (-1.53/1000 * x ** 3) + (4.21/1000000 * x ** 4)) # 238 - (1.48 * x) + (6.91/1000 * x ** 2)


        if self.calibration_mode:
            self.thrower_speed = self.calibration_speed

        # self.thrower_speed = 269.5 + (-2.89 * x) + (0.0209 * x ** 2)

        # self.thrower_speed = 198

        if x > 119:
            self.thrower_speed = 170
        elif x <= 25:
            self.thrower_speed = 250
        else:
            self.thrower_speed = round(330 + (-5.45 * x) + (0.0626 * x ** 2) + (-2.43/10000 * x ** 3)) + 2
            if self.thrower_speed >= 215:
                self.thrower_speed += 12

        global start_time
        current_time = time.time()

        if self.throwing_state >= 5:  # Intermediate state (between real states)
            if current_time - 1.9 >= start_time and self.throwing_state == 7:  # Finish the throwing procedure
                self.throwing_state = 4
            elif current_time - 1.5 >= start_time and self.throwing_state == 6:  # Finish moving
                self.throwing_state = 3
            elif current_time - 0.7 >= start_time and self.throwing_state == 5:  # Send the motor signals and thrower speed again
                self.throwing_state = 2

        elif self.throwing_state == 1:
            print("Initializing thrower..")
            print("Thrower speed:", self.thrower_speed)
            self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            self.throwing_state = 5

        elif self.throwing_state == 2:

            print("Basket diameter:", x)
            self.mainboard.send_motors([0, -0.6, 0])
            # self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            self.throwing_state = 6

        elif self.throwing_state == 3:
            self.mainboard.send_motors([0, 0, 0])
            print("Motors stopped!")
            self.throwing_state = 7

        elif self.throwing_state == 4:
            self.mainboard.send_thrower(100)
            self.throwing_state = 0
            self.counter = 0
            self.find_ball = True
            print("Throwing finished!")

    def set_calibration_speed(self, speed):
        if not self.calibration_mode:
            print("INFO: Calibration mode not set. Speed changes currently have no effect.")
        self.calibration_speed = speed

    def set_calibration_mode(self, flag):
        if not isinstance(flag, bool):
            raise RuntimeError("ERROR: Calibration mode must be set with a boolean value. Value " + str(
                flag) + " is not boolean type.")
        self.calibration_mode = flag
