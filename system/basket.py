import numpy as np


class Basket:
    def __init__(self, thresh_file):
        self.x = 320
        self.y = 0
        self.diameter = 0

        with open(thresh_file) as file:
            f = list(file)
            f = [x.strip() for x in f]
            #                                    lh,        lS,        lV
            self.thresh_min_limits = np.array([int(f[0]), int(f[1]), int(f[2])])
            #                                    hH,        hS,        hV
            self.thresh_max_limits = np.array([int(f[3]), int(f[4]), int(f[5])])

        print("Class Basket initialised.")

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def set_xy(self, xy):
        self.x = xy[0]
        self.y = xy[1]

    def set_x(self, x):
        self.x = x

    def set_diameter(self, diameter):
        self.diameter = diameter

    def get_diameter(self):
        return self.diameter
