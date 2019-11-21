import serial
import time

ser = serial.Serial("COM13", 9600, timeout=0.01)

while True:

    field_id = "A"
    robot_id = "A"

    cmd = input("1 = ping all; 2 = ping ID; 3 = start all; 4 = start ID; 5 = stop all; 6 = stop ID: ")
    send = ""
    if cmd == "":
        break
    elif int(cmd) == 1:
        send = "a" + field_id + "XPING-----"
    elif int(cmd) == 2:
        send = "a" + field_id + robot_id + "PING-----"
        # for i in range(10):
        #     print("Sent:", send)
        #     ser.write(send.encode("utf-8"))
        #     time.sleep(1)
    elif int(cmd) == 3:
        send = "a" + field_id + "XSTART----"
    elif int(cmd) == 4:
        send = "a" + field_id + robot_id + "START----"
    elif int(cmd) == 5:
        send = "a" + field_id + "XSTOP-----"
    elif int(cmd) == 6:
        send = "a" + field_id + robot_id + "STOP-----"

    send = send.encode("utf-8")
    print("Sent:", send)

    ser.write(send)
    time.sleep(0.01)
    if ser.in_waiting > 0:
        print(ser.read(20).decode().strip())
    time.sleep(0.5)
