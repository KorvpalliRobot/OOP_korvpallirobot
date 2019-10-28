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

        # Variables related to balls and basket
        self.ball_x = 320
        self.ball_y = 0
        self.basket_x = 320

        # Variables related to camera
        self.img_center = 320
        self.img_height = 480
        self.ball_y_stop = 350

        # All movement speeds
        self.rotation_speed = 0.03
        self.rotation_speed_basket = 0.2
        self.movement_speed = 0.6
        # Single wheel speed
        self.wheel_speed = 3

        # Proportional controller values
        self.gain_ball = 0.4
        self.gain_basket = 30
        self.gain_movement = 0.01
        self.hysteresis = 7
        self.error_movement = [0, 0, 0, 0, 0]

        # Variables related to other flow logic
        self.find_ball = True
        self.counter = 0
        self.sign = 1
        # A variable so we don't print stuff 60x per second..
        printer_counter = 0

    def autopilot(self):
        print("Starting autopilot.")

        # Main loop
        while True:

            # Fetch data about frame, balls and basket.
            self.img_center, self.img_height = self.camera.find_objects()
            ball_x = self.balls.get_x()
            ball_y = self.balls.get_y()
            basket_x = self.basket.get_x()
            # print("BASKET:", basket_x)

            # Find the vertical stop value independent of frame height.
            self.ball_y_stop = 0.73 * self.img_height

            # If the stop flag has not been set, the robot will stay operational.
            if not self.stop_flag.is_set():

                # If the robot is autonomous, the robot will execute its game logic.
                if self.autonomy.is_set():

                    # NB! Not sure if necessary..
                    # print("Counter:", counter)
                    if self.counter >= 15:
                        self.counter = 0
                        self.find_ball = not self.find_ball

                    # print("BALL: (" + str(ball_x) + "; " + str(ball_y) + ")")

                    # If we haven't found, rotated and moved to the ball:
                    print("Is ball found?")
                    if self.find_ball:
                        # print("Ball is not found.")
                        if self.ball_x > self.img_center:
                            self.sign = 1
                        else:
                            self.sign = -1

                        # Do until ball is in front of us (ball_y > ...)
                        # print("Is ball close enough to robot?")

                        self.rotate_move_to_ball()

                    # If we have found the ball and want to rotate to the basket:
                    else:
                        self.rotate_to_basket()

    def rotate_move_to_ball(self):
        if self.ball_y > self.ball_y_stop:
            print("Ball is close enough to robot!")
            error = abs((self.ball_x - self.img_center) / self.img_center)

            if abs(self.img_center - self.ball_x) < self.hysteresis:
                print("Y-coord is good and ball is centered.")
                self.motors = [0, 0, 0]
                self.counter += 1
                print(self.counter)
                # find_ball = False
            else:
                print("Y-coord is good and ball is not centered.")
                self.counter = 0
                self.motors = [0, 0, self.sign * self.rotation_speed + self.sign * self.gain_ball * error]

        # If the ball is NOT in front of us.
        else:
            # Field of view ~ 90 degrees
            # 45 is in the middle
            # So we can convert pixels to degrees:
            # ball_degrees = ball_position compared to image center divided by
            # a constant, which is the ration between pixels and degrees
            print("Ball is not close enough to robot!")

            print("Set motors to drive to ball.")
            self.error_movement.append(abs(self.ball_y_stop - self.ball_y))
            self.error_movement = self.error_movement[1:]

            ball_degrees = (self.ball_x - self.img_center) / 7.11
            ball_degrees_rad = radians(ball_degrees)
            # Define y_speed as constant, because we always need to move forward
            # Then based on the angle we can calculate the x_speed
            y_speed = self.movement_speed * 1 * self.gain_movement * sum(self.error_movement) \
                      / len(self.error_movement)
            x_speed = tan(ball_degrees_rad) * y_speed

            self.motors = [-x_speed, -y_speed, 0]
        # Send motor speeds to rotate to the closest ball
        print("Move robot!")
        self.mainboard.send_motors(self.motors)

    def rotate_to_basket(self):
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
        if self.basket_x > self.img_center:
            self.sign = 1
        else:
            self.sign = -1

        # Calculate the error for P-controller
        error = abs((self.basket_x - self.img_center) / self.img_center)

        # If the basket is in the center of our view
        print("Is basket centered?")
        if abs(self.img_center - self.basket_x) < self.hysteresis:
            print("Basket centered!")
            # If the ball is NOT in the center of our view then find ball again
            print("Is ball still in center?")
            if abs(self.img_center - self.ball_x) > self.hysteresis or self.ball_x == 0:
                print("Ball not in center!")
                self.find_ball = True
                return
            # motors = [0, 0, 0]
            print("Ball in center!")

            print("Is ball centered for enough time?")
            if self.counter > 10:
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

                self.counter = 0
                self.find_ball = True
            else:
                print("Ball is not centered for enough time. Increasing counter.")
                self.counter += 1
            # find_ball = True

        # If basket not in center
        else:
            print("Basket is not centered.")

            def estimate_distance(size):
                if size != 0:
                    return 59.97575225 * size ** (-1.398997)
                return 0

            # self.mainboard.send_motors_raw([0, wheel_speed * sign + sign * gain_basket * error, 0])

            self.counter = 0

            self.rotation_speed_basket = 0.2

            size = self.balls.get_size()

            translational_speed = estimate_distance(size) * self.rotation_speed_basket * 12

            # print("X_speed:", translational_speed, "Rot:", rotation_speed)
            print("Rotating around the ball.")
            motors = [translational_speed, 0, self.rotation_speed_basket]
            self.mainboard.send_motors(motors)

            # self.rotate_around_ball()
            find_ball = True
            # send_to_mainboard([0, 10, 0])
            # motors = [sign * movement_speed, 0, sign * rotation_speed]
            # if abs(img_center - ball_x) > 10:
            #    self.rotateManual();
            # self.omniDirectional(self.balls.get_x(), self.balls.get_y())

            # if basket_x - img_center > 20:
            #    self.mainboard.send_motors_raw([10, -20, 0])
            # elif basket_x - img_center < 20:
            #    self.mainboard.send_motors_raw([-10, 20, 0])


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
        #print(message)
        self.send_to_mainboard(message)

    # Method to send motor speeds to mainboard
    def send_motors(self, motors):
        motors = Motors.get_motor_speeds(motors[0], motors[1], motors[2])
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        #self.__to_mainboard.put(message, timeout=self.__timeout)
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
        self.ser.write(message)
        time.sleep(0.009)

        response = self.poll_mainboard()
        # Referee commands, responding in real-time
        if "ref" in response:
            #print("REFEREE COMMAND!")
            self.ref_response(response)
            # Print error message for debugging
        elif "buffer empty" in response:
            a = None


    # This method in an endless loop??
    def poll_mainboard(self):
        if self.ser.in_waiting:
            line = self.ser.read(20).decode()

            # Dump the message to a queue of messages and also return it for immediate use.
            #self.__from_mainboard.put(line)
            return line

        return "buffer empty"

        # Return error message for debugging
        #return "Input buffer empty!"

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
                    #self.ser.write("sd:0:0:0:0\n".encode("utf-8"))
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
