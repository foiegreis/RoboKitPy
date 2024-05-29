from robokitpy.plot.plot_2d import *
from robokitpy.core.differential_kinematics import *
from robokitpy.models.planar.rrr_planar import RRR_PLANAR

if __name__ == "__main__":

    # 2R planar robot
    model = RRR_PLANAR()

    # Joint configuration
    thetalist = np.array([np.radians(10), np.radians(50), np.radians(50)])

    # Forward Kinematics
    fk = model.fk(thetalist)
    print("FK ", fk)

    # Jacobian
    J = model.jacobian(thetalist)
    print("J ", J)

    # Differential Forward Kinematics
    thetadot_1 = np.array([1, 0, 1])
    dfk = differential_fk(J, thetadot_1)
    print("DFK ", dfk)

    # Velocity and Force ellipses
    plot_robot_2d(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=True)