import serial
from serial.tools import list_ports
from math import *
import queue
import time
import cv2
import numpy as np


class Robot:
    def __init__(self, mainboard, autonomy, stop_flag, balls, basket):
        self.name = "Placeholder.robot"
        self.mainboard = mainboard
        self.autonomy = autonomy
        self.stop_flag = stop_flag
        self.balls = balls
        self.basket = basket

    def autopilot(self):
        # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
        # the robot won't move.
        hysteresis = 7

        # The center of our camera image
        img_center = 320

        # Default speed for rotation and movement
        rotation_speed = 0.06
        movement_speed = 0.12
        # Töötav variant:
        # rot = 0.03
        # mov = 0.09

        # Basket or ball
        find_ball = True
        counter = 0

        #camera = Camera(self.basket, self.balls, self.stop_flag)

        while not self.stop_flag.is_set():
            #print("Auto:", self.autonomy.is_set())
            if self.autonomy.is_set():

                #camera.find_objects()

                ball_x = 320#self.balls.get_x()
                ball_y = 0#self.balls.get_y()

                basket_x = 320#self.basket.get_x()

                # print("Counter:", counter)
                if counter >= 15:
                    counter = 0
                    find_ball = not find_ball

                # print("BALL: (" + str(ball_x) + "; " + str(ball_y) + ")")

                # Controller logic:
                # if ball is to our right, rotate right;
                # if it's to our left, rotate left;
                # if it's kind of in the middle, don't do anything (hysteresis)
                if find_ball:
                    if ball_x > img_center:
                        sign = 1
                    else:
                        sign = -1

                    if ball_y > 350:
                        if abs(img_center - ball_x) < hysteresis:
                            motors = [0, 0, 0]
                            counter += 1
                            # find_ball = False
                        else:
                            counter = 0
                            motors = [0, 0, sign * rotation_speed]
                    else:
                        # Field of view ~ 90 degrees
                        # 45 is in the middle
                        # So we can convert pixels to degrees:
                        # ball_degrees = ball_position compared to image center divided by
                        # a constant, which is the ration between pixels and degrees
                        ball_degrees = (ball_x - img_center) / 7.11
                        ball_degrees_rad = radians(ball_degrees)

                        # Define y_speed as constant, because we always need to move forward
                        # Then based on the angle we can calculate the x_speed
                        y_speed = movement_speed * 1
                        x_speed = tan(ball_degrees_rad) * y_speed

                        motors = [-x_speed, -y_speed, 0]
                    # Send motor speeds to rotate to the closest ball
                    self.mainboard.send_motors(motors)

                else:
                    if basket_x > img_center:
                        sign = 1
                    else:
                        sign = -1

                    if abs(img_center - basket_x) < hysteresis:
                        motors = [0, 0, 0]
                        if counter > 10:
                            # Send motor and thrower speeds to mainboard
                            self.mainboard.send_motors([0, -0.5, 0])
                            self.mainboard.send_thrower(250)
                            # counter = 0
                        else:
                            counter += 1
                        # find_ball = True
                    else:
                        counter = 0
                        # send_to_mainboard([0, 10, 0])
                        #motors = [sign * movement_speed, 0, sign * rotation_speed]

                        # Send RAW motor speeds to rotate to the basket
                        self.mainboard.send_motors_raw([0, 20*sign, 0])

        print("Closing robot..")


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
        self.__to_mainboard.put(message)

    # Method to send motor speeds to mainboard
    def send_motors_raw(self, motors):
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        try:
            self.__to_mainboard.put(message, timeout=self.__timeout)
        except queue.Full:
            print("Queue full! Raw motor speeds were not sent!")

    # Method to send motor speeds to mainboard
    def send_motors(self, motors):
        motors = Motors.get_motor_speeds(motors[0], motors[1], motors[2])
        message = ("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ":" + str(
            round(motors[2])) + ":0\n").encode("'utf-8")
        try:
            #self.__to_mainboard.put(message, timeout=self.__timeout)
            self.__to_mainboard.put(message)
        except queue.Full:
            #print("Queue full! Motor speeds were not sent!")
            a = 0

    def send_stop(self):
        message = "sd:0:0:0:0\n".encode("'utf-8")
        self.__to_mainboard.put(message)

    def send_thrower(self, thrower_speed):
        message = ("d:" + str(round(thrower_speed)) + "\n").encode("'utf-8")
        if not self.__to_mainboard.full():
            self.__to_mainboard.put(message)
        else:
            print("Queue full, thrower speed was not sent!")

    # ACTUAL SERIAL COMMUNICATION
    # Method to communicate with the mainboard
    # This will both write and receive messages 60 times per second
    def send_to_mainboard(self):
        while True: #not self.stop_flag.is_set():
            time.sleep(0.009)
            if not self.__to_mainboard.empty():
                command = self.__to_mainboard.get()
                #print("CMD:", command)
                self.ser.write(command)
                time.sleep(0.009)
                #print("Write success")
                response = self.poll_mainboard()
                #print(response)

                # Referee commands, responding in real-time
                if "ref" in response:
                    #print("REFEREE COMMAND!")
                    self.ref_response(response)
                # Print error message for debugging
                elif "buffer empty" in response:
                    #print(response)
                    a = 0

                # In other cases it's not necessary to print out anything, because it well be accessible from the queue.
            #else:
                #self.send_motors([0, 0, 0])
                #print("Queue is empty..")
        print("Closing mainboard communication..")

    # This method in an endless loop??
    def poll_mainboard(self):
        if self.ser.in_waiting > 0:
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
