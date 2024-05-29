from robokitpy.core.dynamics import *
from robokitpy.models.spatial.rrr import RRR

""" Inverse Dynamics of the RRR robot """

if __name__ == "__main__":

    # INVERSE DYNAMICS - Example RRR Robot

    # joint configuration
    thetalist = np.array([0.1, 0.1, 0.1])

    # joint velocities
    dthetalist = np.array([0.1, 0.2, 0.3])

    # Given joint accelerations vector --------------
    ddthetalist = np.array([2, 1.5, 1])
    # -----------------------------------------------

    # gravity vector
    g = np.array([0, 0, -9.8])

    # End effector wrench
    Ftip = np.array([1, 1, 1, 1, 1, 1])

    # mass
    M01 = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.089159],
                    [0, 0, 0, 1]])
    M12 = np.array([[0, 0, 1, 0.28],
                    [0, 1, 0, 0.13585],
                    [-1, 0, 0, 0],
                    [0, 0, 0, 1]])
    M23 = np.array([[1, 0, 0, 0],
                    [0, 1, 0, -0.1197],
                    [0, 0, 1, 0.395],
                    [0, 0, 0, 1]])
    M34 = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.14225],
                    [0, 0, 0, 1]])
    M_list = np.array([M01, M12, M23, M34])

    # spatial inertia matrix
    G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
    G_list = np.array([G1, G2, G3])

    # Screw Axes
    S_list = np.array([[1, 0, 1, 0, 1, 0],
                       [0, 1, 0, -0.089, 0, 0],
                       [0, 1, 0, -0.089, 0, 0.425]])

    taulist = inverse_dynamics(thetalist, dthetalist, ddthetalist,  M_list, G_list, S_list, g, Ftip)
    print("\nInverse Dynamics of the 3R robot: \n", taulist)

