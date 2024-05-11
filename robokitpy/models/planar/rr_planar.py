import numpy as np
from robokitpy.core.base import near_zero


""" RR Planar Robot """


class RR_PLANAR:
    def __init__(self):

        self.name = "RR Planar"
        self.joints_num = 2
        self.joints_type = "RR"

        self._L1 = 1
        self._L2 = 1

    @property
    def L1(self):
        return self._L1

    @property
    def L2(self):
        return self._L2

    @L1.setter
    def L1(self, value):
        assert isinstance(value, int)
        self._L1 = value

    @L2.setter
    def L2(self, value):
        assert isinstance(value, int)
        self._L2 = value

    def fk(self, thetalist):
        """Forward Kinematics"""
        th1, th2 = thetalist
        return np.array([self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2),
               self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2)])

    def joints(self, thetalist):
        """ Joint positions """
        th1, th2 = thetalist
        return np.array([[self.L1 * np.cos(th1),  self.L1 * np.sin(th1)],
                         [self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2),
                          self.L1 * np.sin(th1) + self.L2 * np.sin(th1 + th2)]])

    def jacobian(self, thetalist):
        """Jacobian matrix"""
        th1, th2 = thetalist
        J = np.array([[(-self.L1 * np.sin(th1) - self.L2 * np.sin(th1 + th2)),
                        (-self.L2 * np.sin(th1 + th2))],
                        [(self.L1 * np.cos(th1) + self.L2 * np.cos(th1 + th2)),
                        (self.L2 * np.cos(th1 + th2))]])
        return np.where(near_zero(J), 0, J)


