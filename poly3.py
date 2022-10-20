import numpy as np
from geometry import Geometry

class Poly3(Geometry):
    def __init__(self, s, x, y, hdg, length, aU, bU, cU, dU, aV, bV, cV, dV):

        super().__init__(s, x, y, hdg, length)
        self.aU = aU
        self.bU = bU
        self.cU = cU
        self.dU = dU
        self.aV = aV
        self.bV = bV
        self.cV = cV
        self.dV = dV