import system.motors as Motors
import serial
from serial.tools import list_ports
import time


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


ser = get_mainboard_serial_port(1000)

keyPressed = ""
while keyPressed == "":
    thrower_speed = int(input("Thrower speed: "))
    motors = Motors.Motors.get_motor_speeds(0, -0.6, 0)
    ser.write(("sd:0:0:0:" + str(thrower_speed) + "\n").encode("'utf-8"))
    time.sleep(0.7)
    ser.write(("sd:" + str(motors[0]) + ":" + str(round(motors[1])) + ":" + str(round(motors[2])) + ":" + str(thrower_speed) + "\n").encode("'utf-8"))
    time.sleep(0.7)
    keyPressed = input("Press ENTER to continue... ")

