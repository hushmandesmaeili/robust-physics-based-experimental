class SymbolicTrajectory:
    def __init__(self, x, y, z, x_dot, y_dot, z_dot, alpha, alpha_dot, Omega_R, Omega_L):
        self.x = x
        self.y = y
        self.z = z
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.z_dot = z_dot
        self.alpha = alpha
        self.alpha_dot = alpha_dot
        self.Omega_R = Omega_R
        self.Omega_L = Omega_L