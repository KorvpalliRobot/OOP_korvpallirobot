import system.robot as r
import system.camera as cam
import system.xbee as xb
import external_objects.basket as bask
import external_objects.balls as bal


basket = bask.Basket("thresh/thresh_basket.txt")
balls = bal.Balls("thresh/thresh_ball.txt")
xbee = xb.Xbee()
camera = cam.Camera(basket, balls)
robot = r.Robot()

camera.find_objects(xbee)
