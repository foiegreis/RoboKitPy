from robokitpy.core.dynamics import *
from robokitpy.models.spatial.ur5e import UR5e

""" Forward Dynamics of the RRR Robot """

if __name__ == "__main__":

    # FORWARD DYNAMICS - Example RRR Robot

    # Given joint/torques vector ---------------------
    taulist = np.array([0.5, 0.6, 0.7])
    # ------------------------------------------------

    # joint configuration
    thetalist = np.array([0.1, 0.1, 0.1])

    # joint velocities
    dthetalist = np.array([0.1, 0.2, 0.3])

    # gravity vector
    g = np.array([0, 0, -9.8])

    # End effector wrench
    Ftip = np.array([1, 1, 1, 1, 1, 1])

    # mass
    M01 = np.array([[1, 0, 0,        0],
                    [0, 1, 0,        0],
                    [0, 0, 1, 0.089159],
                    [0, 0, 0,        1]])
    M12 = np.array([[ 0, 0, 1,    0.28],
                    [ 0, 1, 0, 0.13585],
                    [-1, 0, 0,       0],
                    [ 0, 0, 0,       1]])
    M23 = np.array([[1, 0, 0,       0],
                    [0, 1, 0, -0.1197],
                    [0, 0, 1,   0.395],
                    [0, 0, 0,       1]])
    M34 = np.array([[1, 0, 0,       0],
                    [0, 1, 0,       0],
                    [0, 0, 1, 0.14225],
                    [0, 0, 0,       1]])
    M_list = np.array([M01, M12, M23, M34])

    # spatial inertia matrix
    G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
    G_list = np.array([G1, G2, G3])

    # Screw Axes
    S_list = np.array([[1, 0, 1,      0, 1,     0],
                      [0, 1, 0, -0.089, 0,     0],
                      [0, 1, 0, -0.089, 0, 0.425]])

    ddthetalist = forward_dynamics(thetalist, dthetalist, M_list, G_list, S_list, g, Ftip, taulist)
    print("\nForward Dynamics of the 3R robot: \n", ddthetalist)
