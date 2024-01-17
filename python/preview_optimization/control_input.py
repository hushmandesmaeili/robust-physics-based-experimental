# control_input.py
class ControlInput:
    def __init__(self, T, p_0, p_T, r_0, r_T, alpha_ddot):
        self.T = T
        self.p_0 = p_0
        self.p_T = p_T
        self.r_0 = r_0
        self.r_T = r_T
        self.alpha_ddot = alpha_ddot

    # Other methods related to ControlInput
