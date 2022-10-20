
class Geometry:

    def __init__(self, s, x, y, hdg, length):
        self.s = s
        self.x = x
        self.y = y
        self.hdg = hdg
        self.length = length
        self.type = None

    def set_params_poly3(self):
