from robokitpy.core.kinematics import *
from robokitpy.models.spatial.ur5e import UR5e


if __name__ == "__main__":

    # UR5e
    # Known joint configuration θ1-θ6
    theta = [0.7854, -0.7854, 0.7854, -1.5708, -1.5708, 0.7854]

    # Model
    model = UR5e()

    # FK DH
    dh_table = model.DH(theta)

    # FORWARD KINEMATICS applying DH
    fk_dh = fk_dh(dh_table)
    print(f"\nForward Kinematics T0{dh_table.shape[0]} applying DH for the configuration {theta}: \n{fk_dh}")

    # FK POE
    M = model.M()
    s_list = model.S()
    b_list = model.B()

    # FORWARD KINEMATICS applying PoE SPACE FORM
    fk_s = fk_space(M, s_list, theta)
    print(f"\nForward Kinematics T0{s_list.shape[0]} applying PoE Space Form for the configuration {theta}: \n{fk_s}")

    # FORWARD KINEMATICS applying PoE BODY FORM
    fk_b = fk_body(M, b_list, theta)
    print(f"\nForward Kinematics T0{b_list.shape[0]} applying PoE Space Form for the configuration {theta}: \n{fk_b}")