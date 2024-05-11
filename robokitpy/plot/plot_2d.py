import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from robokitpy.core.ellipsoid import *


""" Functions to plot robot and velocity and force ellipsoids in 2D """


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
        ax.plot(xs, ys, 'b-o')

    # Plot the end-effector marker -------------------------------------
    start_pos = joints[-2]
    end_effector_pos = joints[-1]
    direction = np.array(end_effector_pos) - np.array(start_pos)
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm != 0 else direction
    ax.quiver(end_effector_pos[0], end_effector_pos[1], direction[0], direction[1],
            angles='xy', scale_units='xy', scale=1, color='r')

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

    #lin_vel_1 = plt.quiver(x2, y2, dfk_1[0], dfk_1[1], angles='xy', scale_units='xy', scale=1, color='r', alpha=0.75)
    #lin_vel_2 = plt.quiver(x2, y2, dfk_2[0], dfk_2[1], angles='xy', scale_units='xy', scale=1, color='g', alpha=0.75)
    #lin_vel_3 = plt.quiver(x2, y2, dfk_3[0], dfk_3[1], angles='xy', scale_units='xy', scale=1, color='b', alpha=0.75)

    # Show the plot ---------------------------------------------------------
    plt.xlim(-2, 3.8)
    plt.ylim(-1.5, 2.5)
    ax.set_aspect('equal', adjustable='box')
    plt.grid(True, linestyle='-')  # '--' specifies dashed lines
    plt.title('2R Robotic Arm')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.show()






