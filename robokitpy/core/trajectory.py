import numpy as np
import matplotlib.pyplot as plt

""" Functions for generating and plotting trapezoidal, cubic polynomial and quintic polynomial trajectories, 
as point-to-point or viapoint"""


def plot_trajectory_profiles(times, positions, velocities, accelerations):
    """ Plots joint trajectory profiles.

    :param times: Array of time values.
    :param positions: Array of joint positions.
    :param velocities: Array of joint velocities.
    :param accelerations: Array of joint accelerations.
    """

    fig, axs = plt.subplots(3, 3, figsize=(13, 8), sharex=True)
    labels = ['Position', 'Velocity', 'Acceleration']
    colors = ['red', 'green', 'blue']

    for i, (vals, label) in enumerate(zip([positions, velocities, accelerations], labels)):
        for j in range(3):
            try:
                axs[i][j].plot(times, vals[:, j], color=colors[i])
            except:
                axs[i][j].plot(times, [v[j] for v in vals], color=colors[i])  # Extract jth element from each sublist

            axs[i][j].set_ylabel(f'{label} (rad/s^2)' if i == 2 else f'{label} (rad/s)')
            axs[i][j].set_title(f'Joint{j + 1} {label}')
            axs[i][j].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()


def generate_p2p_trajectory(profile, q0, qf, t0, tf, N, max_vel=None, acc_time=None, dec_time=None, plot=False):
    """
    Generates a trajectory based on the specified profile.

    :param profile: Trajectory profile type ('cubic', 'quintic', 'trapezoidal').
    :param q0: Initial joint configuration.
    :param qf: Final joint configuration.
    :param t0: Initial time.
    :param tf: Final time.
    :param N: Number of points in the trajectory.
    :param max_vel: Maximum velocity for trapezoidal profile.
    :param acc_time: Acceleration time for trapezoidal profile.
    :param dec_time: Deceleration time for trapezoidal profile.
    :param plot: Boolean indicating whether to plot the trajectory profiles.
    :return: Array of joint positions.
    """

    if profile == 'cubic':
        times, positions, velocities, accelerations = cubic_trajectory(q0, qf, t0, tf, N)

    elif profile == 'quintic':
        times, positions, velocities, accelerations = quintic_trajectory(q0, qf, t0, tf, N)

    elif profile == 'trapezoidal':
        times, positions, velocities, accelerations = trapezoidal_trajectory(q0, qf, t0, tf, N,
                                                                             max_vel, acc_time, dec_time)

    else:
        raise ValueError("Invalid trajectory profile")

    if plot:
        # Plot trajectory profiles
        plot_trajectory_profiles(times, positions, velocities, accelerations)

    return positions


def generate_viapoint_trajectory(profile, q0, qf, viapoints, t0, tf, N, max_vel=None, acc_time=None, dec_time=None, plot=False):
    """
    Generate a trajectory with via-points.

    :param profile: A string indicating the type of trajectory profile to use ('cubic', 'quintic', or 'trapezoidal').
    :param q0: Initial joint configuration.
    :param qf: Final joint configuration.
    :param viapoints: List of via-points through which the trajectory must pass.
    :param t0: Initial time.
    :param tf: Final time.
    :param N: Number of points to generate for the entire trajectory.
    :param max_vel: Maximum velocity for the trapezoidal profile (required if profile is 'trapezoidal').
    :param acc_time: Acceleration time for the trapezoidal profile (required if profile is 'trapezoidal').
    :param dec_time: Deceleration time for the trapezoidal profile (required if profile is 'trapezoidal').
    :param plot: Boolean flag indicating whether to plot the trajectory profiles (default is False).

    :return: A tuple containing lists of joint positions, velocities, and accelerations.
    """

    waypoints = [q0] + viapoints + [qf]
    num_waypoints = len(waypoints)
    if num_waypoints < 2:
        raise ValueError("At least two waypoints are required for via-point trajectory.")

    if profile == 'cubic':
        # Generate trajectory between via-points using cubic polynomials
        times, positions, velocities, accelerations = [], [], [], []
        for i in range(num_waypoints - 1):
            via_points_times, via_points_positions, via_points_velocities, via_points_accelerations = cubic_trajectory(
                waypoints[i], waypoints[i + 1], t0, tf, N // (num_waypoints - 1))
            times.extend(via_points_times)
            positions.extend(via_points_positions)
            velocities.extend(via_points_velocities)
            accelerations.extend(via_points_accelerations)

    elif profile == 'quintic':
        # Generate trajectory between via-points using quintic polynomials
        times, positions, velocities, accelerations = [], [], [], []
        for i in range(num_waypoints - 1):
            via_points_times, via_points_positions, via_points_velocities, via_points_accelerations = quintic_trajectory(
                waypoints[i], waypoints[i + 1], t0, tf, N // (num_waypoints - 1))
            times.extend(via_points_times)
            positions.extend(via_points_positions)
            velocities.extend(via_points_velocities)
            accelerations.extend(via_points_accelerations)

    elif profile == 'trapezoidal':
        # Generate trajectory between via-points using trapezoidal profile
        times, positions, velocities, accelerations = [], [], [], []
        for i in range(num_waypoints - 1):
            via_points_times, via_points_positions, via_points_velocities, via_points_accelerations = trapezoidal_trajectory(
                waypoints[i], waypoints[i + 1], t0, tf, N // (num_waypoints - 1), max_vel, acc_time, dec_time)
            times.extend(via_points_times)
            positions.extend(via_points_positions)
            velocities.extend(via_points_velocities)
            accelerations.extend(via_points_accelerations)

    else:
        raise ValueError("Invalid trajectory profile")

    if plot:
        # Plot trajectory profiles
        plot_trajectory_profiles(times, positions, velocities, accelerations)

    return positions


def trapezoidal_trajectory(q0, qf, t0, tf, N, vmax, acc_time, dec_time):
    """
    Generates a trapezoidal trajectory.

    :param q0: Initial joint configuration.
    :param qf: Final joint configuration.
    :param t0: Initial time.
    :param tf: Final time.
    :param N: Number of points in the trajectory.
    :param vmax: Maximum velocity.
    :param acc_time: Acceleration time.
    :param dec_time: Deceleration time.
    :return: Tuple of arrays containing time, joint positions, velocities, and accelerations.
    """

    # Acceleration and deceleration rates
    a = vmax / acc_time
    d = vmax / dec_time

    times = np.linspace(t0, tf, N)
    positions = np.zeros((N, len(q0)))
    velocities = np.zeros((N, len(q0)))
    accelerations = np.zeros((N, len(q0)))

    constant_time = tf - t0 - acc_time - dec_time
    if constant_time < 0:
        raise ValueError("The sum of acceleration and deceleration times is greater than the total time.")

    for i, t in enumerate(times):
        if t < t0:
            # Before start
            positions[i] = q0
            velocities[i] = 0
            accelerations[i] = 0

        elif t0 <= t < t0 + acc_time:
            # Acceleration phase
            dt = t - t0
            positions[i] = q0 + 0.5 * a * dt ** 2
            velocities[i] = a * dt
            accelerations[i] = a

        elif t0 + acc_time <= t < t0 + acc_time + constant_time:
            # Constant velocity phase
            dt = t - (t0 + acc_time)
            positions[i] = q0 + 0.5 * a * acc_time ** 2 + vmax * dt
            velocities[i] = vmax
            accelerations[i] = 0

        elif t0 + acc_time + constant_time <= t <= tf:
            # Deceleration phase
            dt = t - (t0 + acc_time + constant_time)
            positions[i] = qf - 0.5 * d * (dec_time - dt) ** 2
            velocities[i] = d * (dec_time - dt)
            accelerations[i] = -d
        else:
            # After ebd
            positions[i] = qf
            velocities[i] = 0
            accelerations[i] = 0

    return times, positions, velocities, accelerations


def cubic_trajectory(q0, qf, t0, tf, num_points):
    """
    Generates a cubic trajectory.

    :param q0: Initial joint configuration.
    :param qf: Final joint configuration.
    :param t0: Initial time.
    :param tf: Final time.
    :param num_points: Number of points in the trajectory.
    :return: Tuple of arrays containing time, joint positions, velocities, and accelerations.
    """
    A = np.array([[1, t0, t0**2, t0**3],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [0, 1, 2*tf, 3*tf**2]])

    # TODO: Specify velocities
    dq0 = [0]*len(q0)
    dqf = [0]*len(qf)

    b = np.array([q0, qf, dq0, dqf])
    coeffs = np.linalg.solve(A, b)

    times = np.linspace(t0, tf, num_points)

    T = times[:, None]
    positions = coeffs[0] + coeffs[1] * T + coeffs[2] * T ** 2 + coeffs[3] * T ** 3
    velocities = coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * T ** 2
    accelerations = 2 * coeffs[2] + 6 * coeffs[3] * T

    return times, positions, velocities, accelerations


def quintic_trajectory(q0, qf, t0, tf, num_points):
    """
    Generates a quintic trajectory.

    :param q0: Initial joint configuration.
    :param qf
    :param t0: Initial time.
    :param tf: Final time.
    :param num_points: Number of points in the trajectory.
    :return: Tuple of arrays containing time, joint positions, velocities, and accelerations.
    """
    A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                  [1, tf, tf**2, tf**3, tf**4, tf**5],
                  [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                  [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                  [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                  [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

    # TODO Specify Velocities and Accelerations
    dq0 = [0]*len(q0)
    dqf = [0]*len(qf)
    ddq0 = [0]*len(q0)
    ddqf = [0]*len(qf)

    b = np.array([q0, qf, dq0, dqf, ddq0, ddqf])
    coeffs = np.linalg.solve(A, b)

    times = np.linspace(t0, tf, num_points)
    T = times[:, None]
    positions = coeffs[0] + coeffs[1] * T + coeffs[2] * T ** 2 + coeffs[3] * T ** 3 + coeffs[4] * T ** 4 + coeffs[5] * T ** 5
    velocities = coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * T ** 2 + 4 * coeffs[4] * T ** 3 + 5 * coeffs[5] * T ** 4
    accelerations = 2 * coeffs[2] + 6 * coeffs[3] * T + 12 * coeffs[4] * T ** 2 + 20 * coeffs[5] * T ** 3

    return times, positions, velocities, accelerations





