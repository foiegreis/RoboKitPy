from robokitpy.plot.plot_2d import *
from robokitpy.core.differential_kinematics import *
from robokitpy.models.planar.rr_planar import RR_PLANAR

if __name__ == "__main__":

    # 2R planar robot
    model = RR_PLANAR()

    # Joint configuration
    thetalist = np.array([np.radians(70), np.radians(70)])

    # Forward Kinematics
    fk = model.fk(thetalist)
    print("\nFK\n", fk)

    # Jacobian
    J = model.jacobian(thetalist)
    print("\nJ\n", J)

    # Differential Forward Kinematics
    thetadot_1 = np.array([1, 0])
    dfk = differential_fk(J, thetadot_1)
    print("\nDFK\n", dfk)

    # Velocity and Force ellipses
    plot_robot_2d(model, thetalist, velocity_ellipsoid=True, force_ellipsoid=True)