from robokitpy.plot.plot_2d import *
from robokitpy.core.differential_kinematics import *
from robokitpy.models.planar.rr_planar import RR_PLANAR

if __name__ == "__main__":

    model = RR_PLANAR()

    conf_show = [(85, -170), (70, -140), (60, -120), (50, -100), (40, -80), (30, -60), (20, -40), (10, -20), (5, -10)]
    conf_show = [(np.radians(x), np.radians(y)) for (x, y) in conf_show]

    conf = [np.radians(x) for x in np.linspace(85, 50, 30)]
    conf2 = [np.radians(y) for y in np.linspace(-170, -10, 30)]
    conf_all = [(x, y) for x, y in zip(conf, conf2)]

    # Plots robots, velocity/force ellipsoids and mu_3 measure
    plot_robot_2d_measures(model, conf_all, conf_show, velocity_ellipsoid=False, force_ellipsoid=True,
                           mu1=True, mu2=True, mu3=True)