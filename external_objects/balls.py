import numpy as np


class Balls:
    def __init__(self, thresh_file):
        self.x = None
        self.y = None

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
