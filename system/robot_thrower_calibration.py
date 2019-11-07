import serial
from serial.tools import list_ports
from math import *
import queue
import time

import cv2
import numpy as np


class Robot:
    def __init__(self, mainboard, camera, autonomy, stop_flag, balls, basket, q_thrower_speed):
        self.name = "Placeholder.robot"
        self.mainboard = mainboard
        self.camera = camera
        self.autonomy = autonomy
        self.stop_flag = stop_flag
        self.balls = balls
        self.basket = basket
        self.q_thrower_speed = q_thrower_speed

    def autopilot(self):
        # Hysteresis is the "deadzone" of our controller, that is, if the error is +/- hysteresis value,
        # the robot won't move.
        hysteresis = 7
        print("Starting autopilot.")
        # The center of our camera image
        img_center = 320
        ball_y_stop = 350

        gain_ball = 0.4
        gain_basket = 20
        gain_movement = 0.01
        # When to stop moving (distance from the center)
        offset = 0.015

        # Default speed for rotation and movement
        rotation_speed = 0.03
        #movement_speed = 0.6
        min_speed = 0.06
        movement_speed = min_speed + 0.2
        # Additional variable for correcting the rotation.
        correction = 0
        radius = -10

        error_movement = [0, 0, 0, 0, 0]
        # Single wheel speed
        wheel_speed = 3
        # Töötav variant:
        # rot = 0.03
        # mov = 0.09

        # Basket or ball
        find_ball = True
        counter = 0

        thrower_speed = 175
        printer_counter = 0

        while True:
            self.camera.find_objects()
            ball_x = self.balls.get_x()
            ball_y = self.balls.get_y()
            basket_x = self.basket.get_x()

            if not self.q_thrower_speed.empty():
                thrower_speed = self.q_thrower_speed.get()

            if printer_counter > 60:
                print("d:", self.basket.get_diameter())
                print("Thrower:", thrower_speed)
                printer_counter = 0
            else:
                printer_counter += 1

            if not self.stop_flag.is_set():
                #print("Auto:", self.autonomy.is_set())

                # Analytical formula for thrower speed:
                x = self.basket.get_diameter()
                #thrower_speed = int(0.02 * x ** 2 - 2.31 * x + 231)
                # 0.0028x^4 - 0.2232x^3 + 8.8708x^2 - 174.3115x + 1539.3801
                #thrower_speed = int(-0.0000134995*x**5 + 0.0027670462*(x**4) - 0.2232343543*(x**3) + 8.8708*(x**2) - 174.3115*x + 1539.3801)
                #thrower_speed = int(-0.0049*(x**3) + 0.6311*(x**2) - 26.674*x + 543.7782)

                if self.autonomy.is_set():


                    # Send motor and thrower speeds to mainboard
                    self.mainboard.send_thrower(thrower_speed)
                    time.sleep(0.7)
                    self.mainboard.send_motors([0, -0.6, 0])
                    self.mainboard.send_thrower(thrower_speed)
                    self.autonomy.clear()
                    time.sleep(0.02)
                    #self.mainboard.send_motors([0, -0.6, 0])
                    #self.mainboard.send_thrower(thrower_speed)
                    time.sleep(0.5)
            else:
                return



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
