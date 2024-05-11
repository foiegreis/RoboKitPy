import numpy as np
from robokitpy.models.spatial.robot import Robot


""" UR5e Robot"""


class UR5e(Robot):
    def __init__(self):
        super().__init__()

        self.name = "UR5e"
        self.joints_num = 6
        self.joints_type = "RRRRRR"

        # links specifications
        self.W1 = 0.109
        self.W2 = 0.082
        self.L1 = 0.425
        self.L2 = 0.392
        self.H1 = 0.089
        self.H2 = 0.095

    def M(self):
        """M home configuration htm matrix"""
        return np.array([[-1, 0, 0, (self.L1 + self.L2)],
                  [0, 0, 1, (self.W1 + self.W2)],
                  [0, 1, 0, (self.H1 - self.H2)],
                  [0, 0, 0, 1]])

    def S(self):
        """Screw axes in Space Form"""
        return np.array([[0, 0, 1, 0, 0, 0],
                       [0, 1, 0, -self.H1, 0, 0],
                       [0, 1, 0, -self.H1, 0, self.L1],
                       [0, 1, 0, -self.H1, 0, (self.L1 + self.L2)],
                       [0, 0, -1, -self.W1, (self.L1 + self.L2), 0],
                       [0, 1, 0, (self.H2 - self.H1), 0, (self.L1 + self.L2)]])

    def B(self):
        """Screw axes in Space Form"""
        return np.array([[0, 1, 0, self.W1 + self.W2, 0, self.L1 + self.L2],
                       [0, 0, 1, self.H2, -self.L1 - self.L2, 0],
                       [0, 0, 1, self.H2, -self.L2, 0],
                       [0, 0, 1, self.H2, 0, 0],
                       [0, -1, 0, -self.W2, 0, 0],
                       [0, 0, 1, 0, 0, 0]])

    def DH(self, thetalist):
        """DH Parameters Table: | alpha - a - d - phi |"""
        return np.array([[np.pi / 2, 0, self.H1, -thetalist[0] - np.pi / 2],
                             [0, -self.L1, 0, thetalist[1]],
                             [0, -self.L2, 0, thetalist[2]],
                             [np.pi / 2, 0, self.W1, thetalist[3]],
                             [-np.pi / 2, 0, self.H2, thetalist[4]],
                             [0, 0, self.W2, thetalist[5]]])

