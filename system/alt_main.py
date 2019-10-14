import robot as r
import camera as cam
import basket as bask
import balls as ball
import threading
import remote_control



def main():
    # DATA
    # Holds the information about game logic state (game or manual)
    autonomy = threading.Event()
    autonomy.set()
    # Stop signal for all threads
    stop_flag = threading.Event()
    stop_flag.clear()

    # OBJECTS
    basket = bask.Basket("thresh/thresh_basket.txt")
    balls = ball.Balls("thresh/thresh_ball.txt")

    #xbee = xb.Xbee()
    camera = cam.Camera(basket, balls, stop_flag)
    mainboard = r.Mainboard(autonomy, stop_flag)
    robot = r.Robot(mainboard, camera, autonomy, stop_flag, balls, basket)

    # The main loop for our program, use to display values etc
    while True:
        robot.autopilot()
        # Check for stop signals
        if stop_flag.is_set():
            print("Closing main.py..")
            return

        #print(q_ball.get())


main()