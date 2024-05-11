from plot.plot_3d import *
from models.spatial.ur5e import UR5e

if __name__ == '__main__':

    thetalist = np.array([np.pi/4, -np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4])

    model = UR5e()

    plot_robot(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=True, linear=False, scale=0.1)
