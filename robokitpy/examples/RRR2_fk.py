from robokitpy.core.kinematics import *
from robokitpy.models.spatial.rrr2 import RRR_coplanar


if __name__ == "__main__":

    # EXAMPLE: 3R Spatial Robot

    # Known joint configuration
    theta = [0.92519754, 0.58622516, 0.68427316]

    # model
    model = RRR_coplanar()
    M = model.M()
    s_list = model.S()
    b_list = model.B()

    # FORWARD KINEMATICS applying PoE SPACE FORM of 3R robot
    fk_s = fk_space(M, s_list, theta)
    print(f"\nForward Kinematics of 3R robot T0{s_list.shape[0]} applying PoE Space Form for the configuration {theta}: \n{fk_s}")

    # FORWARD KINEMATICS applying PoE BODY FORM
    fk_b = fk_body(M, b_list, theta)
    print(f"\nForward Kinematics of 3R robot T0{b_list.shape[0]} applying PoE Body Form for the configuration {theta}: \n{fk_b}")
