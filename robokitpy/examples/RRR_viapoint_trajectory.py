import numpy as np
from robokitpy.models.spatial.rrr import RRR
from robokitpy.plot.plot_3d_trajectories import PlotTrajectory
from robokitpy.core.trajectory import generate_viapoint_trajectory

""" Example of generating a viapoint trajectory for a 3D RRR robot, and plotting the robot in 3D"""

if __name__ == '__main__':

    model = RRR()

    # Initial and Final Joint Angles
    q0 = np.array([0.5, -0.6, 1.0])
    qf = np.array([1.57, 0.5, -2.0])

    # Intermediate waypoints
    via_points = [np.array([1.0, -0.2, 0.8]), np.array([1.4, 0.0, 0.0])]

    t0, tf = 0, 8
    N = 100
    # for trapezoidal trajectory
    max_vel = 2
    acc_time = 2
    dec_time = 2

    trajectory_type = 'quintic'
    trajectory = generate_viapoint_trajectory(trajectory_type, q0, qf, via_points, t0, tf, N,
                                               max_vel=max_vel, acc_time=acc_time, dec_time=dec_time,
                                               plot=True)

    plot_robot = PlotTrajectory(model, q0, qf, via_points)
    plot_robot.plot_robot_trajectory(trajectory)