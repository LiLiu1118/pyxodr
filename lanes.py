
class LaneOffset:
    def __init__(self, s, a, b, c, d):
        self.s = s
        self.a = a
        self.b = b
        self.c = c
        self.d = d


class LaneSection:
    def __init__(self, s):
        self.s = s
        self.lane_minus1 = None


class Lane:
    def __init__(self, id):
        self.id = id
        self.width = None

class Lanes:
    def __init__(self):
        self.laneOffsets = []
        self.laneSections = []

class Lane_minus1:
    def __init__(self):
        self.id = -1
        self.width_items = []

class Width_item:
    def __init__(self, sOffset, a, b, c, d):
        self.sOffset = sOffset
        self.a = a
        self.b = b
        self.c = c
        self.d = d