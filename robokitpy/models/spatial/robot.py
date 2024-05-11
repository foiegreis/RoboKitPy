from abc import ABC, abstractmethod


""" Spatial robot abstract class """


class Robot(ABC):
    def __init__(self):
        self.name = None
        self.joints_num = None
        self.joints_type = None

    @abstractmethod
    def M(self):
        """
        Abstract method to return the home configuration matrix of the robot.
        """
        pass

    @abstractmethod
    def S(self):
        """
        Abstract method to return the screw axis of the robot.
        """
        pass

    @abstractmethod
    def B(self):
        """
        Abstract method to return the body-fixed screw axes.
        """
        pass

    @abstractmethod
    def DH(self, thetalist):
        """
        Abstract method to return Denavit-Hartenberg parameters | alpha - a - d - phi  |
.
        """
        pass