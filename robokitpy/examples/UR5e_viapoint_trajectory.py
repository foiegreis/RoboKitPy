import numpy as np
from robokitpy.models.spatial.ur5e import UR5e
from robokitpy.plot.plot_3d_trajectories import PlotTrajectory
from robokitpy.core.trajectory import generate_viapoint_trajectory

""" Example of generating a viapoint trajectory for a 3D UR5e robot, and plotting the robot in 3D"""

if __name__ == '__main__':

    model = UR5e()

    # Initial and Final Joint Angles
    q0 = [0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0]
    qf = [np.pi / 2, -np.pi / 4, np.pi / 4, -np.pi / 2, np.pi / 2, -np.pi / 2]

    # Define via-points
    via_points = [
        [np.pi / 6, -np.pi / 3, np.pi / 4, -np.pi / 3, -np.pi / 3, np.pi / 6],  # Via-point 1
        [np.pi / 4, -np.pi / 6, np.pi / 6, -np.pi / 4, -np.pi / 6, np.pi / 4],  # Via-point 2
        [np.pi / 3, -np.pi / 4, np.pi / 5, -np.pi / 5, -np.pi / 4, np.pi / 3]  # Via-point 3
    ]

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