from robokitpy.core.kinematics import *
from robokitpy.models.spatial.rrr2 import RRR_coplanar


if __name__ == "__main__":

    # Desired end-effector pose
    Tsd = np.array([[-0.585, -0.811, 0, 0.076],
                    [0.811, -0.585, 0, 2.608],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Initial guess
    thetalist0 = np.array([np.pi/4, np.pi/4, np.pi/4])

    # Thresholds
    eps_w = 0.001
    eps_v = 0.0001

    # model
    model = RRR_coplanar()
    M = model.M()
    s_list = model.S()
    b_list = model.B()

    max_iterations = 20

    print("\nInverse Kinematics in Body Form of the 3R robot")
    theta_list, success = ik_body(M, b_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
    print("theta list: ", theta_list)
    print("success: ", success)

    print("\nInverse Kinematics in Space Form of the 3R robot")
    theta_list, success = ik_space(M, s_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
    print("theta list: ", theta_list)
    print("success: ", success)
