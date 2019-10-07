import numpy as np


class Balls:
    def __init__(self, thresh_file):
        self.x = 0
        self.y = 0
        self.balls = []

        with open(thresh_file) as file:
            f = list(file)
            f = [x.strip() for x in f]
            #              lh,        lS,        lV,        hH,        hS,        hV
            self.thresh_min_limits = np.array([int(f[0]), int(f[1]), int(f[2])])
            self.thresh_max_limits = np.array([int(f[3]), int(f[4]), int(f[5])])

        print("Class Balls initialised.")

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def set_xy(self, xy):
        self.x = xy[0]
        self.y = xy[1]

    # Receives keypoints, sorts them and also changes self.x and self.y to match the closest ball.
    def set_balls(self, keypoints):

        # Clear the old list of balls
        self.balls.clear()

        # Orders the list based on keypoint size
        def comparator(keypoint):
            return keypoint.size

        keypoints.sort(reverse=True, key=comparator)

        for keypoint in keypoints:
            x = int(keypoint.pt[0])
            y = int(keypoint.pt[1])
            self.balls.append((x, y))

        #print(self.balls)
        try:
            self.x = self.balls[0][0]
            self.y = self.balls[0][1]
        except:
            self.balls = [(0, 0)]
            self.x = 0
            self.y = 0
            #print("Ball coordinates not available!")
