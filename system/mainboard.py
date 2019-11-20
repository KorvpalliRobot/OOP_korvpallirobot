import queue
import time
from serial.tools import list_ports
import serial


class Mainboard:
    # Class to communicate with the mainboard.
    # The only class to have direct access to it.

    def __init__(self, autonomy, stop_flag):
        self.name = "Placeholder.mainboard"

        # Queues and variables to hold the information going to and coming from the mainboard
        self.__motors_queue = queue.Queue(1)
        self.__thrower_queue = queue.Queue(1)
        self.__servo_queue = queue.Queue()
        self.stop_flag = stop_flag
        self.__thrower_speed = 100
        self.__motors = [0, 0, 0]
        self.__servo_pulse_width = 0

        # Variables for thread clock
        self.frequency = 120
        self.period = 1000 // self.frequency / 1000  # Expression for getting a full millisecond.

        # Queue timeout
        self.timeout = self.period / 2

        # Field and robot ID
        self.field = "A"
        self.id = "A"

        self.autonomy = autonomy

        # Initialize the serial port
        self.ser = Mainboard.get_mainboard_serial_port(self.timeout)

    @staticmethod
    # Scan for mainboard serial ports
    def get_mainboard_serial_port(timeout):
        ports = list_ports.comports()
        for port in ports:
            try:
                ser = serial.Serial(port.device, 9600, timeout=timeout)
                return ser
            except:
                continue
        raise Exception("Could not find suitable or any USB ports.")

    # COMMUNICATION FROM OTHER CLASSES

    # X-, Y- and rotation speeds for input
    def send_motors(self, motors):
        motors = motors.Motors.get_motor_speeds(motors[0], motors[1], motors[2])

        # For double redundancy
        if not self.__motors_queue.full():
            try:
                self.__motors_queue.put(motors, timeout=self.timeout)
            except queue.Full:
                print("Motors queue currently full!")

    def send_thrower(self, thrower_speed):
        # For double redundancy
        if not self.__thrower_queue.full():
            try:
                self.__thrower_queue.put(thrower_speed, timeout=self.timeout)
            except queue.Full:
                print("Thrower queue currently full!")

    def send_servo(self, pulse_width):
        # For double redundancy
        if not self.__servo_queue.full():
            try:
                self.__servo_queue.put(pulse_width, timeout=self.timeout)
            except queue.Full:
                print("Servo queue currently full!")

    def ramp_up(self):
        self.adjust_thrower_ramp(700, 15)

    def ramp_down(self):
        self.adjust_thrower_ramp(1100, 3)

    def adjust_thrower_ramp(self, value, duration):
        for i in range(duration):
            self.send_servo(value)
            time.sleep(0.01)
        self.send_servo(0)
        print("Servo off.")

    # ACTUAL SERIAL COMMUNICATION
    # Method to communicate with the mainboard
    # Writes and receives messages specified number of times per second (default 120)

    # Auxiliary functions:
    def update_variables(self):
        # Update all the variables
        if not self.__motors_queue.empty():
            # This is purely for double redundancy, should never actually produce an exception.
            try:
                self.__motors = self.__motors_queue.get(timeout=self.timeout)
            except queue.Empty:
                print("Motors queue empty!")
        if not self.__thrower_queue.empty():
            try:
                self.__thrower_speed = self.__thrower_queue.get(timeout=self.timeout)
            except queue.Empty:
                print("Thrower queue empty!")
        if not self.__servo_queue.empty():
            try:
                self.__servo_pulse_width = self.__servo_queue.get(timeout=self.timeout)
            except queue.Empty:
                print("Servo queue empty!")

    def mainboard_response(self):
        response = self.poll_mainboard()
        # Referee commands, responding in real-time
        if "ref" in response:
            # print("REFEREE COMMAND!")
            self.ref_response(response)
            # Print error message for debugging
        elif "buffer empty" in response:
            # Error message here:
            a = None

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
        if robot_id[0] == self.field:
            # Send the acknowledge response first if the command is specific to our robot
            if robot_id[1] == self.id:
                acknowledge = ("rf:a" + self.field + self.id + "ACK-----\n").encode("utf-8")
                self.ser.write(acknowledge)
                time.sleep(self.period)
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

    def poll_mainboard(self):
        if self.ser.in_waiting:
            line = self.ser.read(20).decode()
            # Dump the message to a queue of messages and also return it for immediate use.
            # self.__from_mainboard.put(line)
            return line
        return "buffer empty"

    # Main function
    def send_to_mainboard(self):
        while not self.stop_flag.is_set():
            # Wait a quarter of a period to be safe
            time.sleep(self.period / 4)

            # Fetching updated variables from queues and writing them to instance fields.
            # Because we won't ever communicate directly to mainboard (use queues first and then back-up to variables)
            # we should not ever get any communication or interpreter lock errors either.
            # So...
            self.update_variables()

            # First send the __motors and thrower
            message = ("sd:" + str(round(self.__motors[0])) + ":" + str(round(self.__motors[1])) + ":" + str(
                round(self.__motors[2])) + ":" + str(round(self.__thrower_speed)) + "\n").encode("'utf-8")

            # Write the first message and sleep for a quarter of a period again
            self.ser.write(message)
            time.sleep(self.period / 4)

            # Get the first response
            self.mainboard_response()

            # Wait again
            time.sleep(self.period / 4)

            # Then also send the __servo_pulse_width value if needed
            if self.__servo_pulse_width != 0:
                message = ("sv:" + str(round(self.__servo_pulse_width)) + "\n").encode("'utf-8")
                self.ser.write(message)

            # Wait again
            time.sleep(self.period / 4)

            # If we sent the servo pulse, we must also receive the response
            if self.__servo_pulse_width != 0:
                # Get the second response
                self.mainboard_response()
