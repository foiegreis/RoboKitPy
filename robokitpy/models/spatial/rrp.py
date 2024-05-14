import numpy as np
from robokitpy.models.spatial.robot import Robot


""" RRP robot """
# TODO sostituire con 3R a pag 178

class RRP(Robot):
    def __init__(self):
        super().__init__()

        self.name = "RRP"
        self.joints_num = 3
        self.joints_type = "RRP"

        # links specifications
        self.L1 = 2.0
        self.L2 = 2
        self.L3 = 1

    def M(self):
        """M home configuration htm matrix"""
        return np.array([[0, 0, 1, self.L2 + self.L3],
                         [0, -1, 0, 0],
                         [1, 0, 0, self.L1],
                         [0, 0, 0, 1]])

    def S(self):
        """Screw axes in Space Form"""
        return np.array([[0, 0, 1, 0, 0, 0],
                         [0, -1, 0, self.L1, 0, 0],
                         [0, 0, 0, self.L2, 0, 0]])

    def B(self):
        """Screw axes in Space Form"""
        return np.array([[1, 0, 0, 0, -self.L2 - self.L3, 0],
                         [0, 1, 0, self.L2 + self.L3, 0, 0],
                         [0, 0, 0, 0, 0, -self.L3]])

    def DH(self, thetalist):
        """DH Parameters Table: | alpha - a - d - phi |"""
        return np.array([[-np.pi/2, 0, self.L1, thetalist[0]],
                         [-np.pi/2, self.L2, 0, thetalist[1] - np.pi/2],
                         [0, self.L2 + thetalist[2], 0, 0]])

