from robokitpy.core.kinematics import *

""" Functions to compute Jacobian matrices and Differential Forward Kinematics """


def jacobian_body(b_list, theta_list):
    """Computes the body jacobian given the list of screw axes in body form and the joint configuration
    :param b_list: 6xn matrix of the screw axes in body form (screw axes are the rows)
    :param theta_list: list of the joints configurations
    :return: 6xn jacobian matrix in body form
    """
    # we will compose J by columns
    b_list = np.array(b_list)
    Jb = np.array(b_list.T).copy().astype(float)

    T = np.eye(4)

    for i in range(len(theta_list) - 2, -1, -1):

        b = b_list[i+1, :]
        b_skew = vec6_to_skew4(b * - theta_list[i+1])
        mat_exp = skew4_to_matrix_exp4(b_skew)
        T = np.dot(T, mat_exp)

        adj_T = htm_adj(T)
        J_col = np.dot(adj_T, b_list[i, :])
        Jb[:, i] = J_col
    return np.where(near_zero(Jb), 0, Jb)


def jacobian_space(s_list, theta_list):
    """Computes the space jacobian given the list of screw axes in space form and the joint configuration
    :param s_list: 6xn matrix of the screw axes in space form (screw axes are the rows)
    :param theta_list: list of the joints configurations
    :return: 6xn jacobian matrix in space form
    """

    s_list = np.array(s_list)
    Js = np.array(s_list.T).copy().astype(float)

    T = np.eye(4)
    for i in range(1, len(theta_list)):

        s = s_list[i - 1, :]
        s_skew = vec6_to_skew4(s * theta_list[i - 1])
        mat_exp = skew4_to_matrix_exp4(s_skew)
        T = np.dot(T, mat_exp)

        adj_T = htm_adj(T)
        J_col = np.dot(adj_T, s_list[i, :])
        Js[:, i] = J_col
    return np.where(near_zero(Js), 0, Js)


def differential_fk(J, theta_dot):
    """Computes the Differential Forward Kinematics
    :param J: jacobian matrix
    :param theta_dot: list of joint velocities
    :return: end effector velocities 6x1 vector
    """
    return J @ theta_dot


def is_singularity(J):
    """Evaluates if the Jacobian is at a singularity"""
    return np.linalg.matrix_rank(J) < min(np.shape(J))




