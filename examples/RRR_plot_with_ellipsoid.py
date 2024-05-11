from plot.plot_3d import *

if __name__ == '__main__':

    # Joints configuration
    thetalist = np.array([np.pi/4, -np.pi/4, np.pi/4])

    model = RRR()
    plot_robot(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=True, linear=False, scale=0.1)
