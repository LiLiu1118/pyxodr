
class PlanView:

    def __init__(self):
        self.geometrys = []


class Geometry:
    def __init__(self, s, x, y, hdg, length):
        self.s = s
        self.x = x
        self.y = y
        self.hdg = hdg
        self.length = length
        self.model = None


class ParamPoly3():
    def __init__(self, aU, bU, cU, dU, aV, bV, cV, dV):
        self.aU = aU
        self.bU = bU
        self.cU = cU
        self.dU = dU
        self.aV = aV
        self.bV = bV
        self.cV = cV
        self.dV = dV