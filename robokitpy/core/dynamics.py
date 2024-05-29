from robokitpy.core.base import *

"""Functions to compute Robot's Forward and Inverse Dynamics with Newton-Euler approach- 
Based on modern-robotics package formulation"""


def inverse_dynamics(thetalist, dthetalist, ddthetalist,  M_list, G_list, S_list, g, Ftip):
    """Computes inverse dynamics in the space frame for an open chain robot
    :param thetalist: n-vector of joint variables
    :param dthetalist: n-vector of joint rates
    :param ddthetalist: n-vector of joint accelerations
    :param g: Gravity vector g
    :param Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
    :param Mlist: List of link frames {i} relative to {i-1} at the home position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes in matrix form, where the screw axes are the rows
    :return: The n-vector of required joint forces/torques
    """

    n = len(thetalist)

    Mi = np.eye(4)
    Ai = np.zeros((6, n))
    AdTi = [[None]] * (n + 1)
    Vi = np.zeros((6, n + 1))
    Vdi = np.zeros((6, n + 1))
    Fi = np.array(Ftip).copy()

    Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g)]
    AdTi[n] = htm_adjoint(htm_inverse(M_list[n]))

    taulist = np.zeros(n)

    for i in range(n):
        Mi = np.dot(Mi, M_list[i])

        Ai[:, i] = np.dot(htm_adjoint(htm_inverse(Mi)), np.array(S_list)[i, :])

        AdTi[i] = htm_adjoint(np.dot(skew4_to_matrix_exp4(vec6_to_skew4(Ai[:, i] * -thetalist[i])), htm_inverse(M_list[i])))

        Vi[:, i + 1] = np.dot(AdTi[i], Vi[:, i]) + Ai[:, i] * dthetalist[i]
        Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) + Ai[:, i] * ddthetalist[i] \
                       + np.dot(ad(Vi[:, i + 1]), Ai[:, i]) * dthetalist[i]

    for i in range(n - 1, -1, -1):
        Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) \
             + np.dot(np.array(G_list[i]), Vdi[:, i + 1]) \
             - np.dot(np.array(ad(Vi[:, i + 1])).T, \
                      np.dot(np.array(G_list[i]), Vi[:, i + 1]))
        taulist[i] = np.dot(np.array(Fi).T, Ai[:, i])

    return taulist


def mass_matrix(thetalist, M_list, G_list, S_list):
    """Computes the mass matrix of an open chain robot
    :param thetalist: list of joint variables
    :param M_list: List of link frames i relative to i-1 at the home position
    :param G_list: Spatial inertia matrices Gi of the links
    :param S_list: Screw axes in matrix form, where the screw axes are the rows
    :return: numerical inertia matrix M(thetalist)
    """
    n = len(thetalist)
    g = [0, 0, 0]
    Ftip = [0, 0, 0, 0, 0, 0]

    M = np.zeros((n, n))
    for i in range(n):
        dthetalist = [0] * n
        ddthetalist = [0] * n
        ddthetalist[i] = 1
        M[:, i] = inverse_dynamics(thetalist, dthetalist, ddthetalist, M_list, G_list, S_list, g, Ftip)
    return M


def coriolis_term(thetalist, dthetalist, M_list, G_list, S_list):
    """Computes the Coriolis and centripetal terms in the inverse dynamics for a open-chain robot
    :param thetalist: A list of joint variables
    :param dthetalist: A list of joint rates
    :param M_list: List of link frames i relative to i-1 at the home position
    :param G_list: Spatial inertia matrices Gi of the links
    :param S_list: Screw axes in matrix form, where the screw axes are the rows
    :return: The vector c(thetalist,dthetalist) of Coriolis and centripetal terms
    """
    ddthetalist = [0] * len(thetalist)
    g = [0, 0, 0]
    Ftip = [0, 0, 0, 0, 0, 0]
    return inverse_dynamics(thetalist, dthetalist, ddthetalist, M_list, G_list, S_list, g, Ftip)


def gravity_term(thetalist, g, M_list, G_list, S_list):
    """Computes the joint forces/torques an open chain robot requires to overcome gravity at its configuration
    :param thetalist: A list of joint variables
    :param g: 3-vector for gravitational acceleration
    :param M_list: List of link frames i relative to i-1 at the home position
    :param G_list: Spatial inertia matrices Gi of the links
    :param S_list: Screw axes in matrix form, where the screw axes are the rows
    :return grav: The joint forces/torques required to overcome gravity at thetalist
    """
    n = len(thetalist)
    dthetalist = [0] * n
    ddthetalist = [0] * n
    Ftip = [0, 0, 0, 0, 0, 0]
    return inverse_dynamics(thetalist, dthetalist, ddthetalist, M_list, G_list, S_list, g, Ftip)


def endeffector_forces(thetalist, Ftip, M_list, G_list, S_list):
    """Computes the joint forces/torques an open chain robot requires only to create the end-effector force Ftip
    :param thetalist: A list of joint variables
    :param Ftip: Spatial force applied by the end-effector expressed in frame
                 {n+1}
    :param M_list: List of link frames i relative to i-1 at the home position
    :param G_list: Spatial inertia matrices Gi of the links
    :param S_list: Screw axes in matrix form, where the screw axes are the rows
    :return: The joint forces and torques required only to create the
             end-effector force Ftip
    """
    n = len(thetalist)
    dthetalist = [0] * n
    ddthetalist = [0] * n
    g = [0, 0, 0]
    return inverse_dynamics(thetalist, dthetalist, ddthetalist, M_list, G_list, S_list, g, Ftip)


def forward_dynamics(thetalist, dthetalist, M_list, G_list, S_list, g, Ftip, taulist):
    """Computes forward dynamics in the space frame for an open chain robot
    :param thetalist: A list of joint variables
    :param dthetalist: A list of joint rates
    :param taulist: An n-vector of joint forces/torques
    :param g: Gravity vector g
    :param Ftip: Spatial force applied by the end-effector expressed in frame
                 {n+1}
    :param Mlist: List of link frames i relative to i-1 at the home position
    :param Glist: Spatial inertia matrices Gi of the links
    :param Slist: Screw axes in matrix form, where the screw axes are the rows
    :return: The resulting joint accelerations ddthetalist
    """
    ddthetalist = np.dot(np.linalg.inv(mass_matrix(thetalist, M_list, G_list, S_list)), np.array(taulist) \
                  - coriolis_term(thetalist, dthetalist, M_list, G_list, S_list) \
                  - gravity_term(thetalist, g, M_list, G_list, S_list) \
                  - endeffector_forces(thetalist, Ftip, M_list, G_list, S_list))
    return np.round(ddthetalist, 2)


