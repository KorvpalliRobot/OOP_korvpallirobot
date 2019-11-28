import time
from math import *
import simple_pid

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
        self.ball_y_stop = 340
        # All movement speeds
        self.movement_speed = 0.15
        # Single wheel speed
        self.wheel_speed = 3

        # Proportional controller values
        self.gain_movement = 1.7
        self.hysteresis = 10
        self.hysteresis_basket = 8
        self.error_movement = [0, 0, 0, 0, 0, 0]
        self.size = 0
        self.size_average = [0, 0]
        self.size_error = 0.01

        # PID controller values
        self.ball_integral = 0
        self.ball_derivative = 0

        # Ball rotation
        self.rotation_speed = 0.04
        self.gain_ball = 0.02
        self.ball_k_p = 0.04
        self.ball_k_i = 0.00
        self.ball_k_d = 0.006
        self.ball_error_limit = 10

        # Basket rotation
        # self.rotation_speed_basket_default = 0.03  # Do not change!
        # self.gain_basket = 0.1
        # self.basket_k_p = 0.02
        # self.basket_k_i = 0.00
        # self.basket_k_d = 0.000
        # self.basket_error_limit = 1

        # Ball PID for rotate to basket logic
        self.x_speed = 0.05
        self.gain_ball_x = 0.3
        self.ball_k_p_x = 0.2
        self.ball_k_i_x = 0
        self.ball_k_d_x = 0.009
        self.ball_x_error_limit = 10

        self.y_speed = 0.1
        self.gain_ball_y = 2
        self.ball_k_p_y = 0.6
        self.ball_k_i_y = 0.00
        self.ball_k_d_y = 0.01
        self.ball_y_error_limit = self.y_speed * self.gain_ball_y

        # Create PIDs
        self.ball_PID = simple_pid.PID(self.ball_k_p, self.ball_k_i, self.ball_k_d,
                                       output_limits=(-self.ball_error_limit, self.ball_error_limit))
        # Basket PID not needed
        # self.basket_PID = simple_pid.PID(self.basket_k_p, self.basket_k_i, self.basket_k_d,
        #                                  output_limits=(-self.basket_error_limit, self.basket_error_limit))
        self.ball_PID_x = simple_pid.PID(self.ball_k_p_x, self.ball_k_i_x, self.ball_k_d_x, output_limits=(
            -self.ball_x_error_limit, self.ball_x_error_limit))
        self.ball_PID_y = simple_pid.PID(self.ball_k_p_y, self.ball_k_i_y, self.ball_k_d_y, output_limits=(
            -self.ball_y_error_limit, self.ball_y_error_limit))



        # Best PID values so far:
        # self.rotation_speed = 0.03
        # self.ball_k_p = 0.08
        # self.ball_k_i = 0.0002
        # self.ball_k_d = 0.9

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

                    print("BALL")
                    self.rotate_move_to_ball()

                    # Also reset PIDs
                    # self.basket_PID = simple_pid.PID(self.basket_k_p, self.basket_k_i, self.basket_k_d,
                    #                                  output_limits=(-self.basket_error_limit, self.basket_error_limit))
                    self.ball_PID_x = simple_pid.PID(self.ball_k_p_x, self.ball_k_i_x, self.ball_k_d_x, output_limits=(
                        -self.ball_x_error_limit, self.ball_x_error_limit))
                    self.ball_PID_y = simple_pid.PID(self.ball_k_p_y, self.ball_k_i_y, self.ball_k_d_y, output_limits=(
                        -self.ball_y_error_limit, self.ball_y_error_limit))

                # If we have found the ball and want to rotate to the basket:
                else:
                    print("BASKET")
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

            if self.rotate_counter > 30:
                self.rotate = not self.rotate
                self.rotate_counter = 0
            else:
                self.rotate_counter += 1

        elif self.ball_y > self.ball_y_stop:
            # print("Ball is close enough to robot!")
            error = abs((self.ball_x - self.img_center) / self.img_center)
            error **= 5

            if abs(self.img_center - self.ball_x) < self.hysteresis:
                # print("Y-coord is good and ball is centered.")
                self.motors = [0, 0, 0]
                self.counter += 1
                # print(self.counter)
                # find_ball = False
            else:
                # print("Y-coord is good and ball is not centered.")
                self.counter = 0
                #self.motors = [0, 0, self.sign * self.rotation_speed + self.sign * self.gain_ball * error]
                self.ball_PID.setpoint = self.img_center
                output = self.ball_PID(self.ball_x)
                # print(output)
                self.motors = [0, 0, self.sign * self.rotation_speed + self.sign * abs(output) * self.gain_ball]

        # If the ball is NOT in front of us.
        else:
            # Field of view ~ 90 degrees
            # 45 is in the middle
            # So we can convert pixels to degrees:
            # ball_degrees = ball_position compared to image center divided by
            # a constant, which is the ration between pixels and degrees
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
        if self.basket_x > self.img_center:
            self.sign = 1
        else:
            self.sign = -1

        # Calculate the error for controllers
        error = abs((self.basket_x - self.img_center) / self.img_center)

        # If the basket is in the center of our view
        # print("Is basket centered?")
        if abs(self.img_center - self.basket_x) < self.hysteresis_basket:
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

            # Stop
            self.mainboard.send_motors([0, 0, 0])

            # print("Is ball centered for enough time?")
            if self.counter > 5:
                # print("Ball centered for enough time!")

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
            self.counter = 0

            print("Error:", error)

            # Correction for basket placement in the frame
            self.basket_x -= 2

            if error >= 0.3:
                self.rotation_speed_basket = 0.12
                # self.basket_PID.setpoint = self.img_center

                if error >= 0.5:
                    self.rotation_speed_basket = 0.14

                if error >= 0.8:
                    self.rotation_speed_basket = 0.15

                if error >= 0.9:
                    self.rotation_speed_basket = 0.08

                # Better not use PID here..
                # self.rotation_speed_basket = self.rotation_speed_basket_default * self.sign + self.basket_PID(
                # self.basket_x) * self.gain_basket

                # If we are very far from the basket center, just rotate very quickly
                if error >= 0.98:
                    self.rotation_speed_basket = 0.25

                # Safety feature, if too far off-course, find ball again.
                if abs(self.ball_x - self.img_center) > 25 * self.hysteresis:
                    print("Ball not in center!")
                    self.find_ball = True
                    return

                self.rotation_speed_basket *= self.sign

                self.ball_PID_x.setpoint = self.img_center
                self.ball_PID_y.setpoint = self.ball_y_stop

                x_speed = self.gain_ball_x * self.ball_PID_x(self.ball_x) * abs(self.rotation_speed_basket) * self.sign
                y_speed = self.gain_ball_y * self.ball_PID_y(self.ball_y) * abs(self.rotation_speed_basket)

                # BASKET sign not ball sign!!!!!!
                base_speed_x = self.x_speed * self.sign * abs(self.rotation_speed_basket)
                base_speed_y = copysign(self.y_speed, y_speed) * abs(self.rotation_speed_basket)

                # self.x_speed / self.ball_PID_x(self.ball_x) = self.gain_ball_x * abs(self.rotation_speed_basket)
                base_speed_x += x_speed
                base_speed_y += y_speed

                self.motors = [base_speed_x * self.sign, -base_speed_y, self.rotation_speed_basket]
                print(self.motors)
                self.mainboard.send_motors(self.motors)
            else:  # If the error is sufficiently small, moving just one wheel is considerably more accurate.
                gain_wheel = 85

                wheel_speed_temp = 7
                print("One-wheel:", wheel_speed_temp * gain_wheel * error, error)

                if self.basket_x - self.img_center > self.hysteresis_basket:
                    self.mainboard.send_motors_raw([0, wheel_speed_temp + gain_wheel * error, 0])
                elif self.basket_x - self.img_center < self.hysteresis_basket:
                    self.mainboard.send_motors_raw([0, -wheel_speed_temp - gain_wheel * error, 0])

    def rotate_180_degrees(self):
        self.mainboard.send_motors([0, 0, 1.3])
        time.sleep(0.5)
        self.mainboard.send_motors([0, 0, 0])

    def throwing_logic(self):
        # Epoch time in float seconds
        x = self.camera.get_distance_to_basket()

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

        # self.thrower_speed = 269.5 + (-2.89 * x) + (0.0209 * x ** 2)

        # self.thrower_speed = 198

        # if x > 119:
        #     self.thrower_speed = 170
        # elif x <= 25:
        #     self.thrower_speed = 250
        # else:
        #     self.thrower_speed = round(330 + (-5.45 * x) + (0.0626 * x ** 2) + (-2.43/10000 * x ** 3)) + 2
        #     if self.thrower_speed >= 215:
        #         self.thrower_speed += 12
        # print(self.camera.get_distance_to_basket())
        # if x <= 2:
        #     self.thrower_speed = 11.8*x + 156#160 + 5.73*x + 1.71*x**2
        if self.throwing_state == 1:
            if x <= 1.0:
                self.thrower_speed = 166
            elif x < 1.25:
                self.thrower_speed = 20*x + 146
            elif x < 1.55:
                self.thrower_speed = 10*x + 158
            elif x < 2.15:
                self.thrower_speed = 10.2*x + 158
            elif x <= 2.15:
                self.thrower_speed = 11.8 * x + 156
            elif x < 2.1:
                self.thrower_speed = 10*x + 160
            elif x < 2.75:
                self.thrower_speed = 10*x + 162
            elif x < 3.05:
                self.thrower_speed = 10*x + 161
            elif x < 3.25:
                self.thrower_speed = 10*x + 162
            elif x < 3.55:
                self.thrower_speed = 10*x + 163
            elif x < 3.75:
                self.thrower_speed = 20*x + 129
            elif x < 3.95:
                self.thrower_speed = 10*x + 166
            elif x < 4.15:
                self.thrower_speed = 20*x + 127
            else:
                self.thrower_speed = 211

        # else:
        #     if x < 2.15:
        #         self.thrower_speed = 181
        #     elif x < 2.25:
        #         self.thrower_speed = 182
        #     elif x < 2.35:
        #         self.thrower_speed = 183
        #     elif x < 2.45:
        #         self.thrower_speed = 186
        #     elif x < 2.55:
        #         self.thrower_speed = 187
        #     elif x < 2.65:
        #         self.thrower_speed = 188
        #     elif x < 2.85:
        #         self.thrower_speed = 189
        #     else:
        #         self.thrower_speed = 192

        if self.calibration_mode:
            self.thrower_speed = self.calibration_speed

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
            print("Distance from basket:", x)
            print("Thrower speed:", self.thrower_speed)
            self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            # self.mainboard.send_thrower(self.thrower_speed)
            self.throwing_state = 5

        elif self.throwing_state == 2:

            print("Basket distance:", x)
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
