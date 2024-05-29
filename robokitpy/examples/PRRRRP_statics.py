from robokitpy.core.differential_kinematics import *
from robokitpy.core.statics import *
from robokitpy.models.spatial.prrrrp import PRRRRP


if __name__ == "__main__":

    # current configuration
    thetalist = np.array([0, 0, 0, 0, 0, 0])

    # wrench
    wrench = np.array([0, 1, -1, 1, 0, 0])

    # screw axes in space form
    model = PRRRRP()
    s_list = model.S()

    Js = jacobian_space(s_list, thetalist)
    print(f"\nSpace Jacobian Js: \n{Js}")

    singularity = is_singularity(Js)
    print(f"\nIs the Jacobian Singular at the configuration {thetalist}? \n{'No' if not singularity else 'Yes'}")

    tau = statics(Js, wrench)
    print(f"\nStatics: joint torques and forces for the wrench {wrench}: \n{tau}")

