import serial
from math import *
import time
from serial.tools import list_ports


# Initialize serial
def get_mainboard_serial_port():
    ports = list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 9600, timeout=0.01)
            return ser
        except:
            continue
    raise Exception("Could not find suitable or any USB ports.")


ser = get_mainboard_serial_port()

while True:
    if ser.in_waiting > 0:
        # Read the returned message.
        line = ""
        char = ser.read().decode()
        while char != "\n":
            line += char
            char = ser.read().decode()

        # Print out the returned message and also return it for other usage.
        print(line)
