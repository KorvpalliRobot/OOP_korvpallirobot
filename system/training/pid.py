
class PID:
    def __init__(self, _k_p, _k_i, _k_d, _set_point, _data=None):
        self.k_i = _k_i
        self.k_p = _k_p
        self.k_d = _k_d
        self.set_point = _set_point
        if _data is None:
            data = [(0, _set_point), (0, _set_point), (0, _set_point), (0, _set_point), (0, _set_point)]
        self.data = _data
        self.pid_i = 0

    def update(self, time, value):
        self.data.append((time, value))
        if len(self.data) > 5:
            self.data = self.data[1:]
        return self.get_PID_p() + self.get_PID_i() + self.get_PID_d()

    def calculate_error(self, datapoint=None):
        if datapoint is None:
            return self.set_point - self.data[-1][1]
        return self.set_point - self.datapoint[1]

    def get_delta_error(self, datapoint1=None, datapoint2=None):
        if datapoint1 is None or datapoint2 is None:
            return self.calculate_error() - self.calculate_error(self.data[-2][1])
        return datapoint1[1] - datapoint2[1]

    def get_delta_time(self, datapoint1=None,datapoint2 =None):
        if datapoint1 is None or datapoint2 is None:
            return self.data[-1][0] - self.data[-2][0]
        return datapoint1[0] - datapoint2[0]

    def get_PID_p(self):
        return self.k_p * self.calculate_error()

    def get_PID_d(self):
        return self.k_d * self.get_delta_error() / self.get_delta_time()

    def get_PID_i(self):
        if self.calculate_error() < 10:
            self.pid_i = 0
        else:
            self.pid_i += self.k_i * self.calculate_error()
        return self.pid_i










