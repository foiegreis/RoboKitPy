import numpy as np
from robokitpy.models.spatial.robot import Robot


""" Puma 560 Robot """
# TODO compute M, S_list, B_list


class Puma560(Robot):
    def __init__(self):
        super().__init__()

        self.name = "Puma560"
        self.joints_num = 6
        self.joints_type = "RRRRRR"

        # links specifications
        self.W1 = 0.1501
        self.L1 = 0.6718
        self.L2 = 0.4318
        self.L3 = 0.4318
        self.H1 = 0.0203


    def M(self):
        """M home configuration htm matrix"""
        return None

    def S(self):
        """Screw axes in Space Form"""
        return None

    def B(self):
        """Screw axes in Space Form"""
        return None

    def DH(self, thetalist):
        """DH Parameters Table  - | alpha - a - d - phi  |"""
        return np.array([[np.pi/2, 0, self.L1, thetalist[0]],
                         [0, self.L2, 0, thetalist[1]],
                         [-np.pi/2, self.H1, self.W1, thetalist[2]],
                         [np.pi/2, 0, self.L3, thetalist[3]],
                         [-np.pi/2, 0, 0, thetalist[4]],
                         [0, 0, 0, thetalist[5]]])

