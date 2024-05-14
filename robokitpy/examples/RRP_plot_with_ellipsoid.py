from robokitpy.plot.plot_3d import *
from robokitpy.models.spatial.rrp import RRP


if __name__ == '__main__':

    # Joints configuration
    thetalist = np.array([0, np.pi/2, 2])

    model = RRP()
    plot_robot_3d(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=False, linear=False, scale=0.1)
