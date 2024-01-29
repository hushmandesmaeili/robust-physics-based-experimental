# Define State class
class State:
    def __init__(self, c, c_dot, alpha, alpha_dot, omega_R, omega_L):
        self.c = c
        self.c_dot = c_dot
        self.alpha = alpha
        self.alpha_dot = alpha_dot
        self.omega_R = omega_R
        self.omega_L = omega_L