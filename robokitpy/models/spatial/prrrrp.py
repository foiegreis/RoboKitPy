import numpy as np
from robokitpy.models.spatial.robot import Robot


""" PRRRRP Robot """
# TODO compute DH, B_list, M


class PRRRRP(Robot):
    def __init__(self):
        super().__init__()

        self.name = "PRRRRP"
        self.joints_num = 6
        self.joints_type = "PRRRRP"


    def M(self):
        """M home configuration htm matrix"""
        return None

    def S(self):
        """Screw axes in Space Form"""
        return np.array([[0, 0, 0, 0, 0, 1],
                       [1, 0, 0, 0, 0, 0],
                       [0, 0, 1, 1, 0, 0],
                       [0, 0, 1, 1, 1, 0],
                       [0.7071, -0.7071, 0, 0, 0, -0.7071],
                       [0, 0, 0, 0, 1, 0]])

    def B(self):
        """Screw axes in Space Form"""
        return None

    def DH(self, thetalist):
        """DH Parameters Table  - | alpha - a - d - phi  |"""
        return None

