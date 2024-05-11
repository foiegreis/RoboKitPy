import numpy as np
from robokitpy.core.base import near_zero


""" RRR Planar Robot """


class RRR_PLANAR:
    def __init__(self):

        self.name = "RRR Planar"
        self.joints_num = 3
        self.joints_type = "RRR"

        self._L1 = 1
        self._L2 = 1
        self._L3 = 1

    @property
    def L1(self):
        return self._L1

    @property
    def L2(self):
        return self._L2

    @property
    def L3(self):
        return self._L3

    @L1.setter
    def L1(self, value):
        assert isinstance(value, int)
        self._L1 = value

    @L2.setter
    def L2(self, value):
        assert isinstance(value, int)
        self._L2 = value

    @L3.setter
    def L3(self, value):
        assert isinstance(value, int)
        self._L3 = value

    def fk(self, thetalist):
        """Forward Kinematics"""
        th1, th2, th3 = thetalist
        return np.array([self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2) +
                            self.L3 * np.cos(th1 + th2 + th3),
                         self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2) +
                            self.L3 * np.sin(th1 + th2 + th3)])

    def joints(self, thetalist):
        """Joint positions"""
        th1, th2, th3 = thetalist
        return np.array([[self.L1 * np.cos(th1),  self.L1 * np.sin(th1)],
                         [self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2),
                          self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2)],
                         [self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2) +
                          self.L3 * np.cos(th1 + th2 + th3),
                          self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2) +
                          self.L3 * np.sin(th1 + th2 + th3)]
                         ])

    def jacobian(self, thetalist):
        """Jacobian matrix"""
        th1, th2, th3 = thetalist
        J = np.array([[-self.L1 * np.sin(th1) - self.L2 * np.sin(th1 + th2) - self.L3 * np.sin(th1 + th2 + th3),
                       -self.L2 * np.sin(th1 + th2) - self.L3 * np.sin(th1 + th2 + th3),
                        - self.L3 * np.sin(th1 + th2 + th3)],
                        [self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2) + self.L3 * np.cos(th1 + th2 + th3),
                         self.L2 * np.cos(th1 + th2) + self.L3 * np.cos(th1 + th2 + th3),
                         self.L3 * np.cos(th1 + th2 + th3)]])
        return np.where(near_zero(J), 0, J)


