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

def sõida(motors):
    time.sleep(0.09)
    ser.write(("sd:" + str(motors[0]) + ":" + str(round(motors[1])) + ":" + str(round(motors[2])) + ":" + str(0) + "\n").encode("'utf-8"))

def viska(motors, thrower_speed):
    time.sleep(0.09)
    ser.write(("sd:" + str(round(motors[0])) + ":" + str(round(motors[1])) + ser.write(("sd:" + str(round(0)) + ":" + str(round(0)) + ":" + str(round(0)) + ":" + str(170) + "\n").encode("'utf-8"))
    ser.write(("d:"+ str(170) + "\n").encode("'utf-8"))

ser = get_mainboard_serial_port(1000)

motors = Motors.Motors.get_motor_speeds(0,-0.1, 0)
try:
    while True:
        #ser.write(("sd:" + str(round(0)) + ":" + str(round(0)) + ":" + str(round(0)) + ":" + str(170) + "\n").encode("'utf-8"))
        sõida(motors)
        time.sleep(1)
        viska(motors, 170)
        time.sleep(0.2)
except KeyboardInterrupt:
    print('interrupted!')
ser.close()

# keyPressed = ""
# while keyPressed == "":
#
#     motors = Motors.Motors.get_motor_speeds(0, -0.5, 0);
#     sõida(motors);
#     time.sleep(0.8)
#     while (True):
#         viska(motors, 170)
#         time.sleep(1)
#         viska(motors, 0)
#         while True:
#             a = 0
# ser.close()
