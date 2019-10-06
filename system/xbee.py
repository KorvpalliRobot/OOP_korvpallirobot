class Xbee:
    def __init__(self):
        self.name = "Placeholder.xbee"
        self.stop_flag = False

    def listen(self):
        xbee = self
        while xbee.stop_flag == False:
            stop_flag = False
