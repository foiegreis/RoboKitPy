from robokitpy.core.kinematics import *
from robokitpy.plot.plot_3d import get_robot_3d
import matplotlib.pyplot as plt

"""Functions to plot the 3D robot following a p2p or viapoint trajectory"""

class PlotTrajectory:
    """A class for plotting 3D trajectories of robotic arms.

    Attributes:
       model: The robotic arm model.
       q0: Initial joint configuration.
       qf: Final joint configuration.
       viapoints: Viapoints joint configuration.
       fig: The matplotlib figure.
       ax: The matplotlib axis for 3D plotting.
       end_effector_positions: List to store end effector positions during plotting.
    """

    def __init__(self, model, q0, qf, viapoints=None):
        """
        Initializes the PlotTrajectory object.

        :param model: The robotic arm model.
        :param q0: Initial joint configuration.
        :param qf: Final joint configuration.
        :param viapoints: Viapoint joint configurations.
        """

        self.q0 = q0
        self.qf = qf
        self.viapoints = viapoints
        self.model = model
        self.fig = plt.figure(figsize=(10, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.end_effector_positions = []  # Store end effector positions
        self.plot_init()

    def plot_init(self):
        """
        Initializes the plot with base plane, initial, and final positions.
        """

        self.ax.clear()
        # Plot a base plane
        x_length = 0.5
        y_length = 0.5
        x = np.linspace(-x_length / 2, x_length / 2, 100)
        y = np.linspace(-y_length / 2, y_length / 2, 100)
        x, y = np.meshgrid(x, y)
        z = np.zeros_like(x)
        self.ax.plot_surface(x, y, z, alpha=0.5, color='g')

        # Plot initial position
        _, _, _, _, T_list_0 = get_robot_3d(self.model, self.q0)
        p_list_0 = [T[:3, 3] for T in T_list_0]
        self.ee0_x, self.ee0_y, self.ee0_z = p_list_0[-1]
        self.ax.plot(self.ee0_x, self.ee0_y, self.ee0_z, 'ro', label='Start')

        # Plot final position
        _, _, _, _, T_list_f = get_robot_3d(self.model, self.qf)
        p_list_f = [T[:3, 3] for T in T_list_f]
        self.eef_x, self.eef_y, self.eef_z = p_list_f[-1]
        self.ax.plot(self.eef_x, self.eef_y, self.eef_z, 'go', label='End')

        # Plot viapoints
        if self.viapoints:
            for i, v in enumerate(self.viapoints):
                _, _, _, _, T_list_v = get_robot_3d(self.model, v)
                p_list_v = [T[:3, 3] for T in T_list_v]
                self.eev_x, self.eev_y, self.eev_z = p_list_v[-1]

                self.ax.plot(self.eev_x, self.eev_y, self.eev_z, 'yo', label=f'Via Point {i}')
                self.ax.text(self.eev_x + 0.1, self.eev_y, self.eev_z - 0.05, f"v{i}", size=10, zorder=1,
                             color='y')

        # Setting plot labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.view_init(elev=20, azim=45)
        self.ax.set_title(f'3D Visualization of the robot trajectory')
        plt.legend()

    def plot_robot_3d_trajectory(self, model, thetalist):
        """
        Plots the trajectory of the robotic arm in 3D space.

        :param model: The robotic arm model.
        :param thetalist: List of joint angles.
        :return: The end effector position.
        """

        self.ax.clear()

        name, joints_num, joints_type, J, T_list = get_robot_3d(model, thetalist)
        # Plot a base plane
        x_length = 0.5
        y_length = 0.5
        x = np.linspace(-x_length / 2, x_length / 2, 100)
        y = np.linspace(-y_length / 2, y_length / 2, 100)
        x, y = np.meshgrid(x, y)
        z = np.zeros_like(x)
        self.ax.plot_surface(x, y, z, alpha=0.5, color='g')

        # Plot the robot
        p_list = [T[:3, 3] for T in T_list]
        start_p_list = p_list[:-1]
        end_p_list = p_list[1:]

        for i, c in enumerate(joints_type):
            xs, ys, zs = zip(start_p_list[i], end_p_list[i])
            if c == 'R':
                self.ax.plot(xs, ys, zs, 'b-o')
            elif c == 'P':
                self.ax.plot(xs, ys, zs, 'b-s')

        # Plot the trajectory of the end effector
        end_effector_pos = p_list[-1]
        self.end_effector_positions.append(end_effector_pos)  # Track end effector position
        if self.end_effector_positions:
            ee_x, ee_y, ee_z = zip(*self.end_effector_positions)
            self.ax.plot(ee_x, ee_y, ee_z, 'm*', label='End Effector Path')

        # Plot initial position
        _, _, _, _, T_list_0 = get_robot_3d(self.model, self.q0)
        p_list_0 = [T[:3, 3] for T in T_list_0]
        self.ee0_x, self.ee0_y, self.ee0_z = p_list_0[-1]
        self.ax.plot(self.ee0_x, self.ee0_y, self.ee0_z, 'ro', label='Start')

        # Plot final position
        _, _, _, _, T_list_f = get_robot_3d(self.model, self.qf)
        p_list_f = [T[:3, 3] for T in T_list_f]
        self.eef_x, self.eef_y, self.eef_z = p_list_f[-1]
        self.ax.plot(self.eef_x, self.eef_y, self.eef_z, 'go', label='End')

        # Plot viapoints
        if self.viapoints:
            for i, v in enumerate(self.viapoints):
                _, _, _, _, T_list_v = get_robot_3d(self.model, v)
                p_list_v = [T[:3, 3] for T in T_list_v]
                self.eev_x, self.eev_y, self.eev_z = p_list_v[-1]

                self.ax.plot(self.eev_x, self.eev_y, self.eev_z, 'yo', label=f'Via Point {i}')
                self.ax.text(self.eev_x + 0.1, self.eev_y , self.eev_z - 0.05, f"v{i}", size=10, zorder=1,
                             color='y')
        # Update the plot
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.view_init(elev=20, azim=45)
        self.ax.set_title(f'3D Visualization of the {name}, {joints_type} robot with {joints_num} dof')
        plt.legend()
        plt.draw()
        plt.pause(0.01)
        return end_effector_pos

    def plot_robot_trajectory(self, trajectory):
        """
        Plots the trajectory of the robotic arm.

        :param trajectory: List of joint configurations over time.
        """
        for q in trajectory:
            self.plot_robot_3d_trajectory(self.model, q)

        self.ax.text2D(0.5, 0.0, "Robot has reached destination", horizontalalignment='center',
                       verticalalignment='bottom',
                       bbox=dict(facecolor='green', alpha=0.5), transform=self.ax.transAxes)

        plt.show(block=True)


