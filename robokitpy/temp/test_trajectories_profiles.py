import numpy as np
import matplotlib.pyplot as plt

"""Functions to compute Point-to-point and Via-point trajectory generation"""


def trapezoidal_traj(t, t0, tf, q0, qf, vmax, acc_time, dec_time):
    """
    Evaluate a trapezoidal trajectory profile at a given time.

    Args:
        t (float): Current time
        q0 (float): Initial position
        qf (float): Final position
        total_time (float): Total time for the trajectory
        acc_time (float): Time spent in acceleration phase
        dec_time (float): Time spent in deceleration phase
        max_velocity (float): Maximum allowed velocity

    Returns:
        float: Position at time t
        float: Velocity at time t
        float: Acceleration at time t
    """
    # Calculate the time for constant velocity
    constant_time = tf - t0 - acc_time - dec_time
    if constant_time < 0:
        raise ValueError("The sum of acceleration and deceleration times is greater than the total time.")

    # Acceleration and deceleration rates
    a = vmax / acc_time
    d = vmax / dec_time

    if t < t0:
        # Before start
        position = q0
        velocity = 0
        acceleration = 0

    elif t0 <= t < t0 + acc_time:
        # Acceleration phase
        dt = t - t0
        position = q0 + 0.5 * a * dt ** 2
        velocity = a * dt
        acceleration = a

    elif t0 + acc_time <= t < t0 + acc_time + constant_time:
        # Constant velocity phase
        dt = t - (t0 + acc_time)
        position = q0 + 0.5 * a * acc_time ** 2 + vmax * dt
        velocity = vmax
        acceleration = 0

    elif t0 + acc_time + constant_time <= t <= tf:
        # Deceleration phase
        dt = t - (t0 + acc_time + constant_time)
        position = qf - 0.5 * d * (dec_time - dt) ** 2
        velocity = d * (dec_time - dt)
        acceleration = -d
    else:
        # After end
        position = qf
        velocity = 0
        acceleration = 0

    return position, velocity, acceleration


def cubic_poly_traj(t, t0, tf, q0, qf, dq0=0, dqf=0):
    """Evaluates the cubic polynomial trajectory at time t.

    :param t : Time at which to evaluate the polynomial
    :param t0, tf: Initial and final time
    :param q0, qf: Initial and Final Positions
    :param dq0, dqf: Initial and Final Velocities

    :return q: Position at time t
    :return dq: Velocity at time t
    :return ddq: Acceleration at time t
    """
    T = t - t0
    M = np.array([
        [1, t0, t0 ** 2, t0 ** 3],
        [1, tf, tf ** 2, tf ** 3],
        [0, 1, 2 * t0, 3 * t0 ** 2],
        [0, 1, 2 * tf, 3 * tf ** 2]
    ])

    q = np.array([q0, qf, dq0, dqf])

    coeffs = np.linalg.solve(M, q)

    a0, a1, a2, a3 = coeffs

    q = a0 + a1 * T + a2 * T ** 2 + a3 * T ** 3
    dq = a1 + 2 * a2 * T + 3 * a3 * T ** 2
    ddq = 2 * a2 + 6 * a3 * T

    return q, dq, ddq


def quintic_poly_traj(t, t0, tf, q0, qf, dq0=0, dqf=0, ddq0=0, ddqf=0):
    """Evaluates the quintic polynomial trajectory at time t.

    :param t : Time at which to evaluate the polynomial
    :param t0, tf: Initial and final time
    :param q0, qf: Initial and Final Positions
    :param dq0, dqf: Initial and Final Velocities
    :param ddq0,ddqf: Initial and Final Accelerations

    :return q: Position at time t
    :return dq: Velocity at time t
    :return ddq: Acceleration at time t
    """
    T = t - t0
    M = np.array([
        [1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
        [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
        [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
        [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
        [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
        [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]
    ])

    q = np.array([q0, qf, dq0, dqf, ddq0, ddqf])

    coeffs = np.linalg.solve(M, q)

    a0, a1, a2, a3, a4, a5 = coeffs

    q = a0 + a1 * T + a2 * T ** 2 + a3 * T ** 3 + a4 * T ** 4 + a5 * T ** 5
    dq = a1 + 2 * a2 * T + 3 * a3 * T ** 2 + 4 * a4 * T ** 3 + 5 * a5 * T ** 4
    ddq = 2 * a2 + 6 * a3 * T + 12 * a4 * T ** 2 + 20 * a5 * T ** 3
    return q, dq, ddq


def generate_profile(profile, sim_time, t0, tf, q0, qf, dq0=0, dqf=0, ddq0=0, ddqf=0, vmax=0, acc_time=0, dec_time=0):

    time = np.linspace(t0, tf, sim_time)
    q_values = []
    dq_values = []
    ddq_values = []

    q, dq, ddq = None, None, None


    for t in time:

        if profile == 'cubic':
            q, dq, ddq = cubic_poly_traj(t, t0, tf, q0, qf, dq0, dqf)
        elif profile == 'quintic':
            q, dq, ddq = quintic_poly_traj(t, t0, tf, q0, qf, dq0, dqf, ddq0, ddqf)
        elif profile == 'trapezoidal':
            q, dq, ddq = trapezoidal_traj(t, t0, tf, q0, qf, vmax, acc_time, dec_time)

        q_values.append(q)
        dq_values.append(dq)
        ddq_values.append(ddq)

    return q_values, dq_values, ddq_values


def plot_profile(sim_time, q_values, dq_values, ddq_values, profile):

    time = np.linspace(t0, tf, sim_time)

    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    axs[0].plot(time, q_values, label='Position')
    axs[0].set_title('Position Profile')
    axs[0].set_ylabel('Position')

    axs[1].plot(time, dq_values, label='Velocity', color='orange')
    axs[1].set_title('Velocity Profile')
    axs[1].set_ylabel('Velocity')

    axs[2].plot(time, ddq_values, label='Acceleration'
                , color='green')
    axs[2].set_title('Acceleration Profile')
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Acceleration')

    plt.suptitle(f'{profile} Trajectory Profiles')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':

    sim_time = 100

    # Set the initial and final conditions for the trajectory
    t0 = 0  # Initial time
    tf = 10  # Final time
    q0 = 0  # Initial position
    qf = 10  # Final position
    dq0 = 0  # Initial velocity
    dqf = 0  # Final velocity
    ddq0 = 0  # Initial Acceleration
    ddqf = 0  # Final Acceleration

    # Cubic Polynomial
    q, dq, ddq = generate_profile('cubic', sim_time, t0, tf, q0=q0, qf=qf, dq0=dq0, dqf=dqf)
    plot_profile(sim_time, q, dq, ddq, profile='Cubic')

    # Quintic Polynomial
    q, dq, ddq = generate_profile('quintic', sim_time, t0, tf, q0=q0, qf=qf, dq0=dq0, dqf=dqf, ddq0=ddq0, ddqf=ddqf)
    plot_profile(sim_time, q, dq, ddq, profile='Quintic')

    # Trapezoidal
    t0 = 0
    tf = 10
    q0 = 0
    qf = 16
    vmax = 2  # maximum velocity
    acc_time = 2  # time spent in acceleration phase
    dec_time = 2  # time spent in deceleration phase

    q, dq, ddq = generate_profile('trapezoidal', sim_time, t0, tf, q0, qf, vmax=vmax, acc_time=acc_time,
                                  dec_time=dec_time)
    plot_profile(sim_time, q, dq, ddq, profile='Trapezoidal')

