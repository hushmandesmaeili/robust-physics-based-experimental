# Define State class
class State:
    def __init__(self, c, c_dot, alpha, alpha_dot, omega_R=None, omega_L=None):
        """
        Initialize the State object.

        Parameters:
        c (float): The position of the robot's COM, c = [x, y, z].
        c_dot (float): The velocity of the robot's COM, c_dot = [x_dot, y_dot, z_dot].
        alpha (float): The orientation of the robot.
        alpha_dot (float): The angular velocity of the robot.
        omega_R (float): The contact polygon for right foot.
        omega_L (float): The contact polygon for left foot.
        """
        self.c = c
        self.c_dot = c_dot
        self.alpha = alpha
        self.alpha_dot = alpha_dot
        self.omega_R = omega_R
        self.omega_L = omega_L