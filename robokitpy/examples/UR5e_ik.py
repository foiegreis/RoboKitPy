from robokitpy.core.kinematics import *
from robokitpy.models.spatial.ur5e import UR5e


if __name__ == "__main__":

    # Desired end-effector pose
    Tsd = np.array([[ 0, -1, 0, 0.4798],
                     [-1, 0, 0, 0.6339],
                     [ 0, 0, -1, 0.3075],
                     [ 0, 0, 0, 1]])
    # Initial guess
    thetalist0 = np.array([np.pi/4, -np.pi/4, np.pi/4, -np.pi/4, -np.pi/4, np.pi/4])

    # Thresholds
    eps_w = 0.001
    eps_v = 0.0001

    # model
    model = UR5e()
    M = model.M()
    s_list = model.S()
    b_list = model.B()

    max_iterations = 20

    # INVERSE KINEMATICS applying PoE SPACE FORM
    ik_s, success = ik_space(M, s_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
    print(f"\nInverse kinematics in Space form: \n{ik_s}")
    print("success: ", success)

    # INVERSE KINEMATICS applying PoE BODY FORM
    ik_b, success = ik_body(M, b_list, Tsd, thetalist0, eps_w, eps_v, max_iterations)
    print(f"\nInverse kinematics in Body form: \n{ik_b}")
    print("success: ", success)

    print("\nExpected result:\n",  np.round([np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, np.pi/4], 8))