from robokitpy.core.differential_kinematics import *
from robokitpy.models.spatial.rrr2 import RRR_coplanar


if __name__ == "__main__":

    # Joint configuration
    thetalist = np.array([0, np.pi / 4, 0])

    model = RRR_coplanar()
    s_list = model.S()
    b_list = model.B()

    Js = jacobian_space(s_list, thetalist)
    print(f"\nSpace Jacobian of the RRR COPLANAR robot for the configuration {thetalist}: \n{Js}")

    Jb = jacobian_body(b_list, thetalist)
    print(f"\nBody Jacobian of the RRR COPLANAR robot for the configuration {thetalist}: \n{Jb}")

    print("\nIs the Jacobian Singular?")
    singularity = is_singularity(Js)
    print(f"{'No' if not singularity else 'Yes'}")
