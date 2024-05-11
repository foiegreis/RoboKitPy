from core.ellipsoid import *
from plot.plot_2d import *
from core.dfk import *
from models.planar.rr_planar import RR_PLANAR

if __name__ == "__main__":

    # 2R planar robot
    model = RR_PLANAR()

    # Joint configuration
    thetalist = np.array([np.radians(-10), np.radians(20)])

    # Forward Kinematics
    fk = model.fk(thetalist)
    print("FK ", fk)

    # Jacobian
    J = model.jacobian(thetalist)
    print("J ", J)

    # Differential Forward Kinematics
    thetadot_1 = np.array([1, 0])
    dfk = differential_fk(J, thetadot_1)
    print("DFK ", dfk)

    # Velocity and Force ellipses
    plot_robot_2d(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=True)