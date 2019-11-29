import numpy as np


class Balls:
    def __init__(self, thresh_file):
        self.x = 320
        self.y = 0
        self.size = 0
        self.img_center = 0
        self.sizes = []
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

    def get_size(self):
        print("TEre")
        if len(self.sizes) > 1 and abs(self.sizes[0] - self.sizes[1]) <= 50:
            print("Ball 1 size and x: ", self.sizes[0],self.balls[0][0] , "Ball 2 size and x:", self.sizes[1],self.balls[1][0])
            if self.img_center - self.balls[0][0] > self.img_center - self.balls[1][0]:
                return self.sizes[1]
        return self.size

    # Receives keypoints, sorts them and also changes self.x and self.y to match the closest ball.
    def set_balls(self, keypoints):

        # Clear the old list of balls
        self.balls.clear()
        self.sizes.clear()

        # Orders the list based on keypoint size
        def comparator(keypoint):
            return keypoint.size

        keypoints.sort(reverse=True, key=comparator)

        for keypoint in keypoints:
            x = int(keypoint.pt[0])
            y = int(keypoint.pt[1])
            self.balls.append((x, y))
            self.sizes.append(keypoint.size)

        #print(self.balls)
        try:
            if len(self.balls) > 1 and abs(self.sizes[0] - self.sizes[1]) <= 2:
                if self.balls[0][0] >= self.balls[1][0]:
                    right_ball = self.balls[0][0]
                    right_ball_keypoint = keypoints[0]
                else:
                    right_ball = self.balls[1][0]
                    right_ball_keypoint = keypoints[1]

                self.x = right_ball[0]
                self.y = right_ball[1]
                self.size = right_ball_keypoint.size
                return
            self.x = self.balls[0][0]
            self.y = self.balls[0][1]
            self.size = keypoints[0].size
        except:
            self.balls = [(0, 0)]
            self.x = 0
            self.y = 0
            self.size = 0
            #print("Ball coordinates not available!")
