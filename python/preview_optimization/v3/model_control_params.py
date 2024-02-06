# Define ControlParams class
class ModelControlParams:
    """
    Class representing control parameters for a specific preview.

    Attributes:
        T_duration (float): Duration of the task.
        p_0 (list): Initial position [p_0x, p_0y].
        p_T (list): Target position [p_Tx, p_Ty].
        r_0 (float): Initial rotation.
        r_T (float): Target rotation.
        alpha_ddot (float): Acceleration.
    """

    def __init__(self, T_duration, p_0 = [0, 0], p_T = [0, 0], r_0 = 0, r_T = 0, alpha_ddot = 0):
        self.T_duration = T_duration
        self.p_0 = p_0
        self.p_T = p_T
        self.r_0 = r_0
        self.r_T = r_T
        self.alpha_ddot = alpha_ddot
