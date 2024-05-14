from robokitpy.models.spatial.ur5e import UR5e
from robokitpy.plot.plot_3d import *


if __name__ == '__main__':

    thetalist = np.array([np.pi/4, -np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4])

    model = UR5e()

    plot_robot_3d(model, thetalist, velocity_ellipsoid=False, force_ellipsoid=True, linear=False, scale=0.1)
