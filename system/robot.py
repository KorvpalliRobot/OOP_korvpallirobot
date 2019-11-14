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

            # print("Set motors to drive to ball.")
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
        # thrower_speed = int(-0.0049 * (x ** 3) + 0.6311 * (x ** 2) - 26.674 * x + 543.7782)

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
        #     print("Thrower:", thrower_speed)
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
            # motors = [0, 0, 0]
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

                    #thrower_speed = 269.5 + (-2.89 * x) + (0.0209 * x ** 2)

                    #thrower_speed = 198

                    print("Thrower speed:", thrower_speed)
                    print("Basket diameter:", x)
                    start_time = time.time()

                    print("Throwing!")
                    # Throwing..
                    # thrower_speed += 20

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
                            self.mainboard.send_thrower(thrower_speed)
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
        self.thrower_speed = 100

        # Queue timeout
        self.__timeout = 0.01

        # Field and robot ID
        self.field = "A"
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
            round(motors[2])) + ":" + str(round(self.thrower_speed)) + "\n").encode("'utf-8")
        # print(message)
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

    def send_thrower_servo_raw(self, pulse_width):
        message = ("sv:" + str(round(pulse_width)) + "\n").encode("'utf-8")
        self.send_to_mainboard(message)

    # ACTUAL SERIAL COMMUNICATION
    # Method to communicate with the mainboard
    # This will both write and receive messages 60 times per second
    def send_to_mainboard(self, message):
        time.sleep(0.009)
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
        # message = ("d:" + str(round(self.thrower_speed)) + "\n").encode("'utf-8")
        # print("Sending thrower")
        # self.ser.write(message)
        # time.sleep(0.009)

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

    def ramp_up(self):
        self.adjust_thrower_ramp(700, 15)

    def ramp_down(self):
        self.adjust_thrower_ramp(1100, 3)

    def adjust_thrower_ramp(self, value, duration):
        for i in range(duration):
            self.send_thrower_servo_raw(value)
            time.sleep(0.01)
        self.send_thrower_servo_raw(0)
        print("Servo off.")

    def ref_response(self, response):
        # Filter the response
        start_index = response.find(":") + 1
        end_index = response.find(">")
        ref_command = response[start_index:end_index]

        ref_command = ref_command.strip("-")
        robot_id = ref_command[1:3]
        data = ref_command[3:12]
        print(response)
        print("ID:", robot_id)
        # If the field matches:
        #print(robot_id[0], self.field)
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
        rbt_spd = Motors.robot_speed(robot_speed_x, -robot_speed_y)
        dir_ang = Motors.robot_direction_angle(robot_speed_x, -robot_speed_y)
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
