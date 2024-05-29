from robokitpy.core.differential_kinematics import *
from robokitpy.models.spatial.ur5e import UR5e


if __name__ == "__main__":

    # Known joint configuration θ1-θ6
    thetalist = np.array([np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, -np.pi/2, np.pi/4])

    # Model
    model = UR5e()
    s_list = model.S()
    b_list = model.B()

    Js = jacobian_space(s_list, thetalist)
    print(f"\nSpace Jacobian of the RRR COPLANAR robot for the configuration {thetalist}: \n{Js}")

    Jb = jacobian_body(b_list, thetalist)
    print(f"\nBody Jacobian of the RRR COPLANAR robot for the configuration {thetalist}: \n{Jb}")

    print("\nIs the Jacobian Singular?")
    singularity = is_singularity(Js)
    print(f"{'No' if not singularity else 'Yes'}")
