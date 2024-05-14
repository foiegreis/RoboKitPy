import numpy as np
from robokitpy.models.spatial.robot import Robot


""" RRR Coplanar robot """
# TODO Trovare model RRR coplanar

class RRR_coplanar(Robot):
    def __init__(self):
        super().__init__()

        self.name = "RRR_coplanar"
        self.joints_num = 3
        self.joints_type = "RRR"

        # links specifications
        self.L1 = 1
        self.L2 = 1
        self.L3 = 1


    def M(self):
        """M home configuration htm matrix"""
        return np.array([[1, 0, 0, self.L1 + self.L2 + self.L3],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    def S(self):
        """Screw axes in Space Form"""
        return np.array([[0, 0, 1, 0, 0, 0],
                       [0, 0, 1, 0, -self.L1, 0],
                       [0, 0, 1, 0, -self.L1 - self.L2, 0]])

    def B(self):
        """Screw axes in Space Form"""
        return np.array([[0, 0, 1, 0, self.L1 + self.L2 + self.L3, 0],
                       [0, 0, 1, 0, self.L2 + self.L3, 0],
                       [0, 0, 1, 0, self.L3, 0]])

    def DH(self, thetalist):
        """DH Parameters Table: | alpha - a - d - phi |"""
        return np.array([[0, self.L1, 0, thetalist[0]],
                         [0, self.L2, 0, thetalist[1]],
                         [0, self.L3, 0, thetalist[2]]])

