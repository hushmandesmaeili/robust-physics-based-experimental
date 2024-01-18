# plant.py
from state_vector import StateVector
from control_input import ControlInput
from symbolic_trajectory import SymbolicTrajectory
from sympy import symbols, Matrix, exp, cos, sin, sqrt, diff


class Plant:
    def __init__(self, plant_parameters):
        # Initialization logic
        _g = 9.81
        _k = 1.0
        _m = 10.0

    def _compute_p(t, T, p_0, p_T):
        """
        Compute the time-varying COP vector p(t).

        Parameters:
        - t (symbol): Time variable.
        - T (float): Control input, phase duration.
        - p_0 (list): Initial COP position.
        - p_T (list): Target COP position.

        Returns:
        - Matrix: COP vector p(t).
        """
        p_x = (t/T)*(p_T[0] - p_0[0]) + p_0[0]
        p_y = (t/T)*(p_T[1] - p_0[1]) + p_0[1]
        
        return Matrix([p_x, p_y])

    def _compute_r(t, T, r_0, r_T):
        """
        Compute the time-varying rest length r(t).

        Parameters:
        - t (symbol): Time variable.
        - T (float): Control input, phase duration.
        - r_0 (float): Initial rest length.
        - r_T (float): Target rest length.

        Returns:
        - Matrix: Rest length r(t).
        """
        r = (t/T)*(r_T - r_0) + r_0
        
        return r

    def stance_dynamics(self, t, current_state, control_input, h_initial):
        """
        Compute the stance dynamics based on the given time, current state, control input, 
        and initial height.

        Parameters:
        - t (symbol): Time variable.
        - current_state (StateVector): Current (initial) state of the system.
        - control_input (ControlInput): Control input parameters.
        - h_initial (float): Initial height at beginning of phase.

        Returns:
        SymbolicTrajectory: New symbolic state trajectory after stance dynamics.
        """
        # t = symbols('t')  # Define symbolic time variable

        g = self._g
        k = self._k
        m = self._m

        # Extract current state variables from StateVector
        x_0 = current_state.x
        y_0 = current_state.y
        z_0 = current_state.z
        x_dot_0 = current_state.x_dot
        y_dot_0 = current_state.y_dot
        z_dot_0 = current_state.z_dot
        alpha_0 = current_state.alpha
        alpha_dot_0 = current_state.alpha_dot
        Omega_R = current_state.Omega_R
        Omega_L = current_state.Omega_L

        # Extract control input parameters
        T_duration = control_input.T
        alpha_ddot = control_input.alpha_ddot
        p_0 = control_input.p_0
        p_T = control_input.p_T
        r_0 = control_input.r_0
        r_T = control_input.r_T

        alpha_freq = sqrt(g / h_initial)
        omega = sqrt(k / m)

        # Compute p(t) and r(t)
        p = self._compute_p(t, T_duration, p_0, p_T)
        r = self._compute_r(t, T_duration, r_0, r_T)

        # Compute stance dynamics
        beta_x_1 = ((x_0 - p_0[0])/2) + ((x_dot_0*T_duration - (p_T[0] - p_0[0])) / (2*alpha_freq*T_duration))
        beta_x_2 = ((x_0 - p_0[0])/2) - ((x_dot_0*T_duration - (p_T[0] - p_0[0])) / (2*alpha_freq*T_duration))
        beta_y_1 = ((y_0 - p_0[1])/2) + ((y_dot_0*T_duration - (p_T[1] - p_0[1])) / (2*alpha_freq*T_duration))
        beta_y_2 = ((y_0 - p_0[1])/2) - ((y_dot_0*T_duration - (p_T[1] - p_0[1])) / (2*alpha_freq*T_duration))

        x_stance = beta_x_1 * exp(alpha_freq*t) + beta_x_2 * exp(-alpha_freq*t) + p[0]
        y_stance = beta_y_1 * exp(alpha_freq*t) + beta_y_2 * exp(-alpha_freq*t) + p[1]
        
        d1 = z_0 - r_0 + g/omega**2
        d2 = z_dot_0/omega - (r_T - r_0)/(T_duration*omega)
        z_stance = d1 * cos(omega*t) + d2 * sin(omega*t) + r + (g/omega**2)

        # c_stance = Matrix([x_stance, y_stance, z_stance])

        x_dot_stance = diff(x_stance, t)
        y_dot_stance = diff(y_stance, t)
        z_dot_stance = diff(z_stance, t)

        # c_dot_stance = Matrix([x_dot_stance, y_dot_stance, z_dot_stance])

        alpha_stance = alpha_0 + alpha_dot_0 * t + (1/2) * alpha_ddot[2] * t**2

        alpha_dot_stance = alpha_dot_0 + alpha_ddot[2] * t  # hard-coded, faster than diff(alpha_stance, t)

        # Return an instance of SymbolicTrajectory
        return SymbolicTrajectory(x_stance, y_stance, z_stance, x_dot_stance, y_dot_stance, z_dot_stance, 
                                  alpha_stance, alpha_dot_stance, Omega_R, Omega_L)

    def flight_dynamics(self, t, current_state, control_input):
        """
        Compute the flight dynamics based on the given time, current state, and control input.

        Parameters:
        - t (symbol): Time variable.
        - current_state (StateVector): Current state of the system.
        - control_input (ControlInput): Control input parameters.

        Returns:
        - SymbolicTrajectory: New symbolic state trajectory after flight dynamics.
        """

        g = self._g

        # Extract current state variables from StateVector
        x_0 = current_state.x
        y_0 = current_state.y
        z_0 = current_state.z
        x_dot_0 = current_state.x_dot
        y_dot_0 = current_state.y_dot
        z_dot_0 = current_state.z_dot
        alpha_0 = current_state.alpha
        alpha_dot_0 = current_state.alpha_dot
        Omega_R = current_state.Omega_R
        Omega_L = current_state.Omega_L

        c_double_dot_gravity = [0, 0, g]
        
        c_flight = Matrix([x_0 + x_dot_0*t + (1/2)*c_double_dot_gravity[0]*t**2,
                            y_0 + y_dot_0*t + (1/2)*c_double_dot_gravity[1]*t**2,
                            z_0 + z_dot_0*t + (1/2)*c_double_dot_gravity[2]*t**2])
        
        c_dot_flight = Matrix([x_dot_0 + c_double_dot_gravity[0]*t,
                            y_dot_0 + c_double_dot_gravity[1]*t,
                            z_dot_0 + c_double_dot_gravity[2]*t])

        alpha_flight = alpha_0 + alpha_dot_0*t

        alpha_flight_dot = alpha_dot_0

        # Return an instance of SymbolicTrajectory
        return SymbolicTrajectory(c_flight[0], c_flight[1], c_flight[2], c_dot_flight[0], c_dot_flight[1], c_dot_flight[2], 
                                  alpha_flight, alpha_flight_dot, 0, 0)
