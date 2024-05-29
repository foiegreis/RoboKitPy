import numpy as np

""" Functions to compute linear algebra """

def near_zero(z):
    """
    Determines whether a scalar is zero
    :param z: scalar
    :return: bool
    """
    return abs(z) < 1e-6


def axis_angle(p):
    """ Computes the axis angle representation of a 3D vector of exponential coordinates
    :param p: 3D vector of exponential coordinates OmegaTheta = [e1, e2, e3]
    :return: axis-angle representation (omega, theta)
    """
    return p / np.linalg.norm(p), np.linalg.norm(p)


def vec3_to_skew3(p):
    """
    Returns the skew symmetric matrix representation [p] of a 3D  vector
    :param p: 3D vector
    :return: 3x3 matrix
    """
    p_sk = np.array([[0, -p[2], p[1]],
                     [p[2], 0, -p[0]],
                     [-p[1], p[0], 0]])
    return p_sk


def vec6_to_skew4(s):
    """
    Returns the skew symmetric matrix representation [S] of a 6D twist vector
    :param s: 6D twist vector s = [omega, v]
    :return: 4x4 matrix [s] = [[ [omega], v],[0, 0]]
    """
    omega = s[0:3]
    v = s[3:]
    p_sk = vec3_to_skew3(omega)
    twist_sk = np.r_[np.c_[p_sk, v.reshape(3, 1)],[[0, 0, 0, 0]]]
    return twist_sk


def skew3_to_vec3(p_skew):
    """Returns the 3D vector of a 3x3 Skew Symmetric Matrix
    :param p: [p] = skew symmetric matrix
    :return : 3D vector
    """
    p = np.r_[[p_skew[2][1], p_skew[0][2], p_skew[1][0]]]
    return p


def skew4_to_vec6(s_skew):
    """Returns the 6D vector of a 4x4 Skew Symmetric Matrix
    :param s_skew: [s] = [[ [omega], v],[0, 0]]
    :return: s = [omega, v]
    """
    s = np.r_[[s_skew[2][1], s_skew[0][2], s_skew[1][0]],[s_skew[0][3], s_skew[1][3], s_skew[2][3]]]
    return s


def htm_to_rp(T):
    """ Extracts R and p from a homogeneous transformation matrix T
    :param T: 4x4 homogeneous transformation matrix
    :return R: 3x3 rotation matrix
    :return p: 3D position vector
    """
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return R, p


def htm_inverse(T):
    """Returns the inverse of a 4x4 homogeneous transformation matrix
    :param T: 4x4 homogeneous transformation matrix in SE(3)
    :return T_trans: 4x4 matrix T-1
    """
    R, p = htm_to_rp(T)
    R_transp = np.array(R).T
    return np.r_[np.c_[R_transp, -np.dot(R_transp, p)], [[0, 0, 0, 1]]]


def htm_adj(T):
    """ Computes the 6X6 skew symmetric adjoint representation of a 4x4 transformation matrix T
    :param T: 4x4 homogeneous transformation matrix in SE(3)
    :return adj_T : 6x6 adjoint representation of T
    """

    R, p = htm_to_rp(T)
    p_sk = vec3_to_skew3(p)
    return np.r_[np.c_[R, np.zeros((3, 3))], np.c_[np.dot(p_sk, R), R]]


def skew3_to_matrix_exp3(p_sk):
    """
    Computes the 3x3 matrix exponential Rot(p_hat, theta) = e^[p]theta of the exp coordinates p_hat*theta, given [p] and theta
    using the Rodrigues formula for rotations
    :param p_sk: 3x3 skew symmetric matrix
    :return: 3x3 matrix exponential
    """

    ptheta = skew3_to_vec3(p_sk) # exponential coordinates OmegaTheta
    if near_zero(np.linalg.norm(ptheta)):
        mat_exp = np.eye(3)
        return mat_exp
    else:
        theta = axis_angle(ptheta)[1]
        p_sk_pure = p_sk / theta
        mat_exp = np.array(np.eye(3) + np.sin(theta) * p_sk_pure + (1 - np.cos(theta)) * np.dot(p_sk_pure, p_sk_pure))
        res = np.where(near_zero(mat_exp), 0, mat_exp)
        return res


def skew4_to_matrix_exp4(s_sk):
    """Computes the matrix exponential of a 4x4 skew matrix representation of a 6D vector (Screw axis)
    :param s_sk: 4X4 skew symmetric matrix [s] = [[omega], v],[0, 0]]
    :return mat_exp: 4X4 transformation matrix
    """

    omegatheta_sk = s_sk[0:3, 0:3]
    omegatheta = skew3_to_vec3(omegatheta_sk)  # 3D vector of exponential coordinates OmegaTheta
    vtheta = s_sk[0:3, 3]

    # Case Prismatic Joint:
    if near_zero(np.linalg.norm(omegatheta)):
        # return [[I, v*theta], [0, 1]]
        return np.r_[np.c_[np.eye(3), omegatheta], [[0, 0, 0, 1]]]

    # Case Revolute Joint
    else:
        # return [[e^[omega]theta, G(theta)v],[0, 1]]
        theta = axis_angle(omegatheta)[1]
        omega_sk = omegatheta_sk / theta
        matexp3 = skew3_to_matrix_exp3(omegatheta_sk)
        G = np.eye(3)*theta + (1 - np.cos(theta))*omega_sk + (theta - np.sin(theta)) * np.dot(omega_sk, omega_sk)
        v = np.dot(G, vtheta)/theta
        matexp4 = np.r_[np.c_[matexp3, v], [[0, 0, 0, 1]]]
        return matexp4


def mat3_to_log3(R):
    """Computes the 3x3 matrix logarithm [omega] of a 3x3 rotation matrix
    :param R: 3x3 rotation matrix in SO(3)
    :returns : 3x3 matrix logarithm in so(3)
    """

    # trR = 1 + 2cos(theta)
    tr_R = np.trace(R)
    cos_theta = (tr_R - 1)/2.0

    # Case mat log undefined
    if cos_theta >= 1:
        return np.zeros((3, 3))

    # Case trR = -1, theta = pi
    elif cos_theta <= -1:
        if not near_zero(1 + R[2][2]):
            omega = (1.0 / np.sqrt(2 * (1 + R[2][2]))) * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not near_zero(1 + R[1][1]):
            omega = (1.0 / np.sqrt(2 * (1 + R[1][1]))) * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omega = (1.0 / np.sqrt(2 * (1 + R[0][0]))) * np.array([1 + R[0][0], R[1][0], R[2][0]])
        mat_log = vec3_to_skew3(np.pi * omega)
        return mat_log

    # Case theta in [0, pi)
    else:
        theta = np.arccos(cos_theta)
        mat_log_theta = (1 / (2 * np.sin(theta))) * (R - np.array(R).T)
        mat_log = mat_log_theta * theta
        return mat_log


def htm_to_log6(T):
    """Computes the 4x4 matrix logarithm [S] of a 4x4 homogeneous transformation matrix T=(R, p)
    :param T: 4x4 homogeneous transformation matrix in SE(3)
    :return : matrix logarithm 6x6 in se(3)
    """

    R, p = htm_to_rp(T)

    omega_sk = mat3_to_log3(R)

    # Case R = I -> omega = 0
    if np.array_equal(omega_sk, np.zeros((3, 3))):
        mat_log = np.r_[np.c_[np.zeros((3, 3)), [T[0][3], T[1][3], T[2][3]]], [[0, 0, 0, 0]]]
        return mat_log
    else:

        theta = np.arccos((np.trace(R) - 1) / 2.0)
        I = np.eye(3)

        # G = 1/theta * I - 1/2 * [omega] + (1/theta - 1/2 * cot(theta/2) * [omega}^2
        G = I - (0.5 * omega_sk) + (((1.0 / theta - 0.5 * (1 / np.tan(theta / 2.0))) * np.dot(omega_sk, omega_sk)) / theta)

        # v = G * p
        v = np.dot(G, p)

        # [S] = [[ [omega], v ],[0, 0]]
        mat_log = np.r_[np.c_[omega_sk, v], [[0, 0, 0, 0]]]

        return mat_log


def angles_and_p_from_T(T):
    """ Returns rotation angles and translation vector given htm T"""
    R = T[:3, :3]
    p = T[:3, 3]
    angle_x = np.arctan2(R[2][1], R[2][2])
    angle_y = np.arctan2(-R[2][0], np.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
    angle_z = np.arctan2(R[1][0], R[0][0])

    return (angle_x, angle_y, angle_z), p


def eigen(M):
    """Returns eigen values and eigen vectors (columns) in descending order
    :param M: square matrix
    :return: eigen_values, eigen_vectors
    """
    eigval, eigvec = np.linalg.eig(M)
    idx = eigval.argsort()[::-1]
    return eigval[idx], eigvec[:, idx]


def htm_adjoint(T):
    """Computes the adjoint representation of a homogeneous transformation matrix
    :param T: A 4 x 4 homogeneous transformation matrix
    :return: The 6x6 adjoint representation [AdT] of T
    """
    R, p = htm_to_rp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                 np.c_[np.dot(vec3_to_skew3(p), R), R]]


def ad(V):
    """Computes the 6x6 matrix [adV] of the given 6-vector - LIE BRACKET
    :param V: A 6-vector spatial velocity
    :return: The corresponding 6x6 matrix [adV]
    """
    omgmat = vec3_to_skew3([V[0], V[1], V[2]])
    return np.r_[np.c_[omgmat, np.zeros((3, 3))],
                 np.c_[vec3_to_skew3([V[3], V[4], V[5]]), omgmat]]