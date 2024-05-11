from .dfk import *


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