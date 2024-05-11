import numpy as np

from .dfk import *


def ellipsoids_2d(J):
    """Computes the matrix A that represents the ellipsoid"""

    A = J @ J.T
    lambd_A, vect_A = eigen(A)

    # Velocity ellipsoid
    axes_A = np.sqrt(lambd_A)

    # Force ellipsoid
    axes_B, vect_B = 1/axes_A, vect_A

    return (axes_A, vect_A), (axes_B, vect_B)


def ellipsoids_3d(J):

    Jw, Jv = J[:3, :], J[3:, :]

    Aw = Jw @ Jw.T
    Av = Jv @ Jv.T

    # Angular velocity ellipsoid
    lambd_Aw, vect_Aw = eigen(Aw)
    axes_Aw= np.sqrt(lambd_Aw)
    print("\nAngular velocity ellipsoid", ellipsoid_measures(lambd_Aw))

    # Linear velocity Ellipsoid
    lambd_Av, vect_Av = eigen(Av)
    axes_Av = np.sqrt(lambd_Av)
    print("\nLinear velocity ellipsoid", ellipsoid_measures(lambd_Av))

    # Angular force ellipsoid
    axes_Bw, vect_Bw = 1/axes_Aw, vect_Aw
    print("\nAngular force ellipsoid", ellipsoid_measures(1/lambd_Aw))

    # Linear force ellipsoid
    axes_Bv, vect_Bv = 1/lambd_Av, vect_Av
    print("\nLinear force ellipsoid", ellipsoid_measures(1/lambd_Av))

    return (axes_Aw, vect_Aw), (axes_Av, axes_Av), (axes_Bw, vect_Bw), (axes_Bv, vect_Bv)


def ellipsoid_measures(lambd):
    """Computes ellipsoid measusres
    :param lambd: eigenvalues"""

    # isotropic test
    mu_1 = np.round(np.sqrt(lambd[0]) / np.sqrt(lambd[-1]), 3)
    # condition number
    mu_2 = np.round(lambd[0] / lambd[1], 3)
    # volume measure
    mu_3 = np.round(np.sqrt(np.prod(lambd)), 3)

    return f"mu_1 {mu_1}, mu_2 {mu_2} mu_3 {mu_3}"
