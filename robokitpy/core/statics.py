import numpy as np

""" Functions to compute statics """


def statics(J, F):
    """Computes the statics equation of the joint torques given the Jacobian Matrix and the
     Wrench of end-effector forces
     param: j = jacobian matrix
     param: f = wrench vector 6x1
     returns: tau = joint forces vector n x 1
     """
    J_trans = np.transpose(J)
    tau = np.dot(J_trans, F)
    return tau