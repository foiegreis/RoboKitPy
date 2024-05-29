from robokitpy.core.base import *
from robokitpy.core.differential_kinematics import *

""" Functions to compute Forward and Inverse Kineamtcis """


def dh_to_htm(alpha=None, a=None, d=None, phi=None):
    """
    Produces the Homogeneous Transformation Matrix corresponding to the Denavit-Hartenberg parameters (alpha, a, d, phi)
    :param alpha: rad
    :param a: float
    :param d: float
    :param phi: rad
    :return: 4x4 homogeneous transformation matrix
    """

    T = np.array([[np.cos(phi), -np.sin(phi)*np.cos(alpha), np.sin(phi)*np.sin(alpha), a*np.cos(phi)],
                  [np.sin(phi), np.cos(phi)*np.cos(alpha), -np.cos(phi) * np.sin(alpha), a * np.sin(phi)],
                  [0, np.sin(alpha), np.cos(alpha), d ],
                  [0, 0, 0, 1]])
    return T


def fk_dh(dh_table, all=False):
    """
    Computes the Forward Kinematics given the DH Table of Denavit Hartenberg Parameters
    :param dh_table: n x 4 np.ndarray
    :return: 4x4 homogeneous transformation matrix
    """

    fk = np.eye(4)
    T_list = [fk]

    for joint in dh_table:
        alpha, a, d, phi = joint
        tj = dh_to_htm(alpha, a, d, phi)
        fk = np.matmul(fk, tj)
        if all:
            T_list.append(np.where(near_zero(fk), 0, np.round(fk, 3)))
    res = np.where(near_zero(fk), 0, np.round(fk, 4))
    if all:
        return res, T_list
    return res


def fk_body(M, b_list, theta_list):
    """
    Computes the Forward Kinematics given the M matrix, the list of screw axes in Body Form and the configuration
    :param M: 4x4 M0b homogeneous transformation matrix of the zero configuration
    :param b_list: nx6 matrix of the body screw axes for the robot joints
    :param theta_list: configuration of the robot i.e. list of joint values
    :return: 4x4 homogeneous transformation matrix of the end effector for the configuration
    """
    T = np.array(M)
    for i, b in enumerate(b_list):
        b_skew = vec6_to_skew4(np.array(b) * theta_list[i])
        mat_exp = skew4_to_matrix_exp4(b_skew)
        T = np.dot(T, mat_exp)

    fk = np.round(np.where(near_zero(T), 0, T), 4)
    return fk


def fk_space(M, s_list, theta_list):
    """
    Computes the Forward Kinematics given the M matrix, the list of screw axes in Space Form and the configuration
    :param M: 4x4 M0b homogeneous transformation matrix of the zero configuration
    :param s_list: nx6 matrix of the space screw axes for the robot joints
    :param theta_list: configuration of the robot i.e. list of joint values
    :return: 4x4 homogeneous transformation matrix of the end effector for the configuration
    """
    T = np.eye(4)
    for i, s in enumerate(s_list):
        s_skew = vec6_to_skew4(np.array(s) * theta_list[i])
        mat_exp = skew4_to_matrix_exp4(s_skew)
        T = np.dot(T, mat_exp)

    fk = np.matmul(T, M)
    res = np.where(near_zero(fk), 0, np.round(fk, 4))
    return res


def joints_position_from_fk(dh_table):

    # Base frame origin
    joints = [np.array([0, 0, 0, 1])]

    # Compute the position of each joint
    fk, T_list = fk_dh(dh_table, all=True)
    for T in T_list:
        joints.append(T[:3, 3])  # Extract the position part of the transformation matrix

    return joints


def ik_body(M, b_list, T_sd, theta_list_0, e_omega, e_v, max_iterations=10):
    """Computes the Inverse Kinematics in body form of an open chain robot
    :param M: 4x4 homogeneous transformation matrix of the home configuration
    :param b_list: nx6 matrix of the screw axes in body form
    :param T_sd: 4x4 homogeneous transformation matrix of the desired end effector pose with respect to space frame s
    :param theta_list_init: 1xn list of the initial guess of joint configurations
    :param e_w: error threshold on the angular velocity
    :param e_v: error threshold on the linear velocity
    :return: IK = 1xn list of the joint configurations that places the end effector in T_sd
    """

    i = 0
    theta_list = np.array(theta_list_0).copy()

    # Ik(theta_0)
    T_sb = fk_body(M, b_list, theta_list)

    T_bs = htm_inverse(T_sb)

    T_bd = np.dot(T_bs, T_sd)

    twist_b_skew = htm_to_log6(T_bd)

    twist_b = skew4_to_vec6(twist_b_skew)

    omega_b = twist_b[0:3]
    v_b = twist_b[3:]
    err = np.any(omega_b > e_omega) or np.any(v_b > e_v)

    while err and i < max_iterations:

        J_b = jacobian_body(b_list, theta_list)
        J_b_pseudoinv = np.linalg.pinv(J_b)
        delta_theta = np.dot(J_b_pseudoinv, twist_b)
        theta_list = theta_list + delta_theta

        i = i + 1

        T_sb = fk_body(M, b_list, theta_list)
        T_bs = htm_inverse(T_sb)
        T_bd = np.dot(T_bs, T_sd)

        twist_b_skew = htm_to_log6(T_bd)
        twist_b = skew4_to_vec6(twist_b_skew)
        omega_b = twist_b[0:3]
        v_b = twist_b[3:]
        err = np.any(omega_b > e_omega) or np.any(v_b > e_v)

    return theta_list, not err


def ik_space(M, s_list, T_sd, theta_list_0, e_omega, e_v, max_iterations=10):
    """Computes the Inverse Kinematics in space form of an open chain robot
       :param M: 4x4 homogeneous transformation matrix of the home configuration
       :param s_list: nx6 matrix of the screw axes in space form
       :param T_sd: 4x4 homogeneous transformation matrix of the desired end effector pose with respect to space frame s
       :param theta_list_init: 1xn list of the initial guess of joint configurations
       :param e_w: error threshold on the angular velocity
       :param e_v: error threshold on the linear velocity
       :return: IK = 1xn list of the joint configurations that places the end effector in T_sd
       """

    i = 0
    theta_list = np.array(theta_list_0).copy()

    # Ik(theta_0)
    T_sb = fk_space(M, s_list, theta_list)
    adj_T_sb = htm_adj(T_sb)

    T_bs = htm_inverse(T_sb)

    T_bd = np.dot(T_bs, T_sd)

    twist_b_skew = htm_to_log6(T_bd)
    twist_b = skew4_to_vec6(twist_b_skew)
    twist_s = np.dot(adj_T_sb, twist_b)

    omega_s = twist_s[0:3]
    v_s= twist_s[3:]
    err = np.any(omega_s > e_omega) or np.any(v_s > e_v)

    while err and i < max_iterations:
        J_s = jacobian_space(s_list, theta_list)
        J_s_pseudoinv = np.linalg.pinv(J_s)
        delta_theta = np.dot(J_s_pseudoinv, twist_s)
        theta_list = theta_list + delta_theta

        i = i + 1

        T_sb = fk_space(M, s_list, theta_list)
        adj_T_sb = htm_adj(T_sb)

        T_bs = htm_inverse(T_sb)

        T_bd = np.dot(T_bs, T_sd)

        twist_b_skew = htm_to_log6(T_bd)
        twist_b = skew4_to_vec6(twist_b_skew)
        twist_s = np.dot(adj_T_sb, twist_b)

        omega_s = twist_s[0:3]
        v_s = twist_s[3:]
        err = np.any(omega_s > e_omega) or np.any(v_s > e_v)

    return theta_list, not err