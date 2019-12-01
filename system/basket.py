import numpy as np


class Basket:
    def __init__(self, thresh_file):
        self.x = 320
        self.y = 0
        self.diameter = 0
        self.true_x = 320
        self.width = 640
        self.is_blue = False

        # Extreme points
        self.l_m = 0
        self.r_m = 0
        self.t_m = 0
        self.b_m = 0

        if "blue" in thresh_file:
            self.is_blue = True

        with open(thresh_file) as file:
            f = list(file)
            f = [x.strip() for x in f]
            #                                    lh,        lS,        lV
            self.thresh_min_limits = np.array([int(f[0]), int(f[1]), int(f[2])])
            #                                    hH,        hS,        hV
            self.thresh_max_limits = np.array([int(f[3]), int(f[4]), int(f[5])])

        print("Class Basket initialised.")

    def get_x(self):
        if self.x == 0 or self.x == self.width:  # If the basket is not visible
            if self.true_x >= self.width / 2:
                return self.width
            else:
                return 0
        else:  # If the basket is visible, return the current x
            if self.is_blue:  # -1 due to thresholding error
                self.x -= 1
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

    def get_distance(self):
        x = self.diameter
        return 5.05 + (-0.0953 * x) + (7.46 / 10000 * x ** 2) + (-2.14 / 1000000 * x ** 3)

    def set_extreme_points(self, extreme_points):
        self.l_m = extreme_points[0]
        self.r_m = extreme_points[1]
        self.t_m = extreme_points[2]
        self.b_m = extreme_points[3]

    def get_extreme_points(self):
        return [self.l_m, self.r_m, self.t_m, self.b_m]
