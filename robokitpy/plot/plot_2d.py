import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from robokitpy.core.ellipsoid import *


""" Functions to plot robot and velocity and force ellipsoids in 2D """
# TODO PLOT PRISMATIC JOINTS

def get_robot_2d(model, thetalist):
    """
    Computes the Forward Kinematics and Jacobian of the Robot, given the model.

    :param model: instance of the Robot class
    :param thetalist: joint configuration
    :return: name, joints_num, joints_type, J, joints
    """

    # Model attributes
    name = model.name
    joints_num = model.joints_num
    joints_type = model.joints_type

    # Fk and joint positions
    fk = model.fk(thetalist)
    print("Forward Kinematics:\n", fk)

    joints = [row for row in model.joints(thetalist)]
    joints.insert(0, np.array([0, 0]))

    # Jacobian
    J = model.jacobian(thetalist)

    return name, joints_num, joints_type, J, joints


def plot_robot_2d(model, thetalist, velocity_ellipsoid=False, force_ellipsoid=False):
    """ Draws the 2D robot and the velocity and force ellipsoids"""

    # Create a figure and an axes
    fig, ax = plt.subplots()

    name, joints_num, joints_type, J, joints = get_robot_2d(model, thetalist)

    # Plot a base line -----------------------------------------------
    x = np.linspace(-1, 1, 100)
    y = np.zeros_like(x)
    ax.plot(x, y, 'g-')  # 'o' for circle markers

    # Plot the robot --------------------------------------------------
    for start, end in zip(joints[:-1], joints[1:]):
        xs, ys = zip(start, end)
        for c in name:
            # revolute joints
            if c == 'R':
                ax.plot(xs, ys, 'b-o')
            # prismatic joints
            if c == 'P':
                ax.plot(xs, ys, 'b-s')

    # Plot the end-effector marker -------------------------------------
    start_pos = joints[-2]
    end_effector_pos = joints[-1]
    direction = np.array(end_effector_pos) - np.array(start_pos)
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm != 0 else direction
    ax.quiver(end_effector_pos[0], end_effector_pos[1], direction[0], direction[1],
            angles='xy', scale_units='xy', color='r')
    # ax.plot(end_effector_pos[0], end_effector_pos[1], 'rD', label='End Effector')  # 'ro' for red circle

    # Plot velocity and force ellipses -----------------------------------
    (axes_A, vect_A), (axes_B, vect_B) = ellipsoids_2d(J)
    angle = np.degrees(np.arctan2(vect_A[1][0], vect_A[0][0]))

    # Add the ellipsoid
    if velocity_ellipsoid:
        ellipse_vel = Ellipse((end_effector_pos[0], end_effector_pos[1]), width=axes_A[0],
                          height=axes_A[1], angle=angle, edgecolor='r', facecolor='none', lw=2)
        ax.add_patch(ellipse_vel)

    if force_ellipsoid:
        ellipse_force = Ellipse((end_effector_pos[0], end_effector_pos[1]), width=axes_B[0]*0.5,
                            height=axes_B[1]*0.5, angle=angle, edgecolor='y', facecolor='none', lw=1)
        ax.add_patch(ellipse_force)

    # Show the plot ---------------------------------------------------------
    ax.xaxis.get_data_interval()
    ax.yaxis.get_data_interval()
    ax.margins(0.1)
    ax.set_aspect('equal', adjustable='box')
    plt.grid(True, linestyle='--')  # '--' specifies dashed lines
    plt.title(f'{name}, {joints_type} 2D Robot with {joints_num} dof')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def plot_robot_2d_measures(model, thetalist_all, thetalist_show, velocity_ellipsoid=False, force_ellipsoid=False,
                           mu1=False, mu2=False, mu3=False):
    """ Draws one or more 2D robots and plots mu1, mu2, mu3 measures for a sequence of configurations """

    # Create a figure and an axes
    fig, ax = plt.subplots(4, 1)
    mu1_list = []
    mu2_list = []
    mu3_list = []

    # Plot a base line -----------------------------------------------
    x = np.linspace(-0.5, 1, 100)
    y = np.zeros_like(x)
    ax[0].plot(x, y, 'g-')  # 'o' for circle markers

    # Get measures --------------------------------------------------
    for thetalist in thetalist_all:
        _, _, _, J, joints = get_robot_2d(model, thetalist)

        mu_1, mu_2, mu_3 = None, None, None

        # Velocity measures
        if velocity_ellipsoid:
            A = J @ J.T
            lambd_A, _ = eigen(A)
            mu_1, mu_2, mu_3 = ellipsoid_measures(lambd_A)

        # Force measures
        if not velocity_ellipsoid and force_ellipsoid:
            A = J @ J.T
            lambd_A, _ = eigen(A)
            mu_1, mu_2, mu_3 = ellipsoid_measures(1/lambd_A)

        mu1_list.append(mu_1)
        mu2_list.append(mu_2)
        mu3_list.append(mu_3)

    # Plot the robot --------------------------------------------------
    for thetalist in thetalist_show:
        _, _, _, J, joints = get_robot_2d(model, thetalist)
        for start, end in zip(joints[:-1], joints[1:]):
            xs, ys = zip(start, end)
            ax[0].plot(xs, ys, 'b-o')

        end_effector_pos = joints[-1]

        # Plot velocity and force ellipses -----------------------------------
        (axes_A, vect_A), (axes_B, vect_B) = ellipsoids_2d(J)
        angle = np.degrees(np.arctan2(vect_A[1][0], vect_A[0][0]))

        # Add the ellipsoid
        if velocity_ellipsoid:
            ellipse_vel = Ellipse((end_effector_pos[0], end_effector_pos[1]), width=axes_A[0],
                              height=axes_A[1], angle=angle, edgecolor='r', facecolor='none', lw=2)
            ax[0].add_patch(ellipse_vel)

        if force_ellipsoid:
            ellipse_force = Ellipse((end_effector_pos[0], end_effector_pos[1]), width=axes_B[0]*0.5,
                                height=axes_B[1]*0.5, angle=angle, edgecolor='y', facecolor='none', lw=1)
            ax[0].add_patch(ellipse_force)

    if mu1:
        i = 1
        plot_measures(ax, 'mu1', 'Axes Ratio', i, len(thetalist_all), mu1_list)
    if mu2:
        i = 2
        plot_measures(ax, 'mu2', 'Condition Number', i, len(thetalist_all), mu2_list)
    if mu3:
        i = 3
        plot_measures(ax, 'mu3', 'Volume ratio',i, len(thetalist_all), mu3_list)

    # Show the plot ---------------------------------------------------------
    scope1 = 'Velocity' if velocity_ellipsoid else ''
    scope2 = 'Force' if force_ellipsoid else ''
    ax[0].set_title(f'{scope1} {scope2} ellipsoids RR 2D Robot with 2 dof')
    ax[0].set_xlabel('x')
    ax[0].set_ylabel('y')
    ax[0].xaxis.get_data_interval()
    ax[0].yaxis.get_data_interval()
    ax[0].margins(0.1)
    ax[0].grid(True, linestyle='--')  # '--' specifies dashed lines
    fig.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1, hspace=0.5, wspace=0.5)
    plt.show()


def plot_measures(ax, name, description, i, n_el, values):
    """ Plots measures"""

    x_list = np.linspace(0, 2, n_el)
    ax[i].plot(x_list, values, 'b-')  # Blue line for the second plot
    ax[i].set_title(f'Ellipsoid {name} measure: {description} ')
    ax[i].set_ylabel(name)
    ax[i].set_xlabel('x')
    ax[i].grid(True, linestyle='--')  # '--' specifies dashed lines







