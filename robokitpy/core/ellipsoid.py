from robokitpy.core.differential_kinematics import *
import numpy as np
import warnings
warnings.filterwarnings('error', category=RuntimeWarning)


""" Functions to compute velocity and force ellipsoids in 2D and 3D """


def ellipsoids_2d(J):
    """Computes the matrix A that represents the ellipsoid"""

    A = J @ J.T
    lambd_A, vect_A = eigen(A)

    # Velocity ellipsoid
    axes_A = np.sqrt(lambd_A)

    # Force ellipsoid
    try:
        axes_B, vect_B = 1/axes_A, vect_A
    except RuntimeWarning:
        axes_B, vect_B = None, None

    return (axes_A, vect_A), (axes_B, vect_B)


def ellipsoids_3d(J):
    """Computes the 3D ellipsoid given the Jacobian matrix J"""

    Jw, Jv = J[:3, :], J[3:, :]

    Aw = Jw @ Jw.T
    Av = Jv @ Jv.T

    # Angular velocity ellipsoid
    lambd_Aw, vect_Aw = eigen(Aw)
    axes_Aw= np.sqrt(lambd_Aw)
    measures_Aw = ellipsoid_measures(lambd_Aw)

    # Linear velocity Ellipsoid
    lambd_Av, vect_Av = eigen(Av)

    axes_Av = np.sqrt(lambd_Av)

    measures_Av = ellipsoid_measures(lambd_Av)

    # Angular force ellipsoid
    try:
        axes_Bw, vect_Bw = 1/axes_Aw, vect_Aw
        measures_Bw = ellipsoid_measures(1 / lambd_Aw)
    except RuntimeWarning:
        axes_Bw, vect_Bw = None, None
        measures_Bw = [None, None, None]

    # Linear force ellipsoid
    try:
        axes_Bv, vect_Bv = 1/lambd_Av, vect_Av
        measures_Bv = ellipsoid_measures(1 / lambd_Av)
    except RuntimeWarning:
        axes_Bv, vect_Bv = None, None
        measures_Bv = [None, None, None]

    print_measures(measures_Aw, measures_Av, measures_Bw, measures_Bv)
    return (axes_Aw, vect_Aw), (axes_Av, axes_Av), (axes_Bw, vect_Bw), (axes_Bv, vect_Bv)


def ellipsoid_measures(lambd):
    """Computes ellipsoid measusres
    :param lambd: eigenvalues"""

    # isotropic test
    try:
        mu_1 = np.round(np.sqrt(lambd[0]) / np.sqrt(lambd[-1]), 2)
    except RuntimeWarning:
        mu_1 = 'inf'
    # condition number
    try:
        mu_2 = np.round(lambd[0] / lambd[1], 2)
    except RuntimeWarning:
        mu_2 = 'inf'
    # volume measure
    try:
        mu_3 = np.round(np.sqrt(np.prod(lambd)), 2)
    except RuntimeWarning:
        mu_3 = 'inf'

    return [mu_1, mu_2, mu_3]


def print_measures(m_Aw, m_Av, m_Bw, m_Bv):
    """Pribts the ellipsoid measures"""

    data = {'Vw': m_Aw, 'Vv': m_Av, 'Fw': m_Bw, 'Fv': m_Bv}
    print("Ellipsoids measures: ")
    print('    | mu1  | mu2  | mu3  | mu4 ')
    for d, k in data.items():
        mu_1, mu_2, mu_3 = k
        print(f' {d} | {mu_1} | {mu_2} | {mu_3} ')
