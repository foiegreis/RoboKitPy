import numpy as np
from robokitpy.models.spatial.ur5e import UR5e
from robokitpy.plot.plot_3d_trajectories import PlotTrajectory
from robokitpy.core.trajectory import generate_p2p_trajectory

""" Example of generating a point-to-point trajectory for a 3D UR5e robot, and plotting the robot in 3D"""

if __name__ == '__main__':

    model = UR5e()

    # Initial and Final Joint Angles
    q0 = [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]
    qf = [np.pi / 2, -np.pi / 4, np.pi / 4, -np.pi / 2, np.pi / 2, -np.pi / 2]

    t0 = 0
    tf = 5
    N = 100

    # For trapezoidal trajectory ------------
    max_vel = 2
    acc_time = 2
    dec_time = 2
    # --------------------------------------

    trajectory_type = 'quintic'
    trajectory = generate_p2p_trajectory(trajectory_type, q0, qf, t0, tf, N,
                                     max_vel=max_vel, acc_time=acc_time, dec_time=dec_time,
                                     plot=True)

    plot_robot = PlotTrajectory(model, q0, qf)
    plot_robot.plot_robot_trajectory(trajectory)