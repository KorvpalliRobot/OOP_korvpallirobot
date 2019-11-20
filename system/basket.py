import numpy as np


class Basket:
    def __init__(self, thresh_file):
        self.x = 320
        self.y = 0
        self.diameter = 0
        self.true_x = 320
        self.width = 640

        with open(thresh_file) as file:
            f = list(file)
            f = [x.strip() for x in f]
            #                                    lh,        lS,        lV
            self.thresh_min_limits = np.array([int(f[0]), int(f[1]), int(f[2])])
            #                                    hH,        hS,        hV
            self.thresh_max_limits = np.array([int(f[3]), int(f[4]), int(f[5])])

        print("Class Basket initialised.")

    def get_x(self):
        if self.x == 0 or self.x == self.width:
            if self.true_x >= self.width / 2:
                return self.width
            else:
                return 0
        else:
            return self.x

    def get_y(self):
        return self.y

    def set_xy(self, xy, width):
        self.x = xy[0]
        self.y = xy[1]
        self.width = width
        if 0 < self.x < width:
            self.true_x = self.x

    def set_x(self, x, width):
        self.x = x
        self.width = width
        if 0 < x < width:
            self.true_x = x

    def set_diameter(self, diameter):
        self.diameter = diameter

    def get_diameter(self):
        return self.diameter
