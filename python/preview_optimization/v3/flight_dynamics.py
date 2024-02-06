import sympy as sp
from state import State
from model_control_params import ModelControlParams

# Define FlightDynamics class
class FlightDynamics:
    def __init__(self, g=9.81):
        self._g = g

    def simulate(self, initial_state: State, control_input: ModelControlParams):
        t = sp.symbols('t')
        c_0, c_dot_0 = initial_state.c, initial_state.c_dot
        alpha0, alphadot0, omega_R, omega_L = initial_state.alpha, initial_state.alpha_dot, initial_state.omega_R, initial_state.omega_L
        x0, y0, z0 = c_0
        xdot0, ydot0, zdot = c_dot_0
        
        T_total = control_input.T_duration

        g = self._g # 
        acc = sp.Matrix([0, 0, g])  # Acceleration [ax, ay, az]

        # Initial state vectors
        c0 = sp.Matrix([x0, y0, z0])  # Initial position
        c0_dot = sp.Matrix([xdot0, ydot0, zdot])  # Initial velocity

        # Equations of motion for flight phase
        c = c0 + c0_dot * t + 0.5 * acc * t**2  # Position vector [x(t), y(t), z(t)]
        c_dot = c0_dot + acc * t  # Velocity vector [xdot(t), ydot(t), zdot(t)]

        # Extracting individual components
        x, y, z = c[0], c[1], c[2]
        xdot, ydot, zdot = c_dot[0], c_dot[1], c_dot[2]

        # Heading equation (remains the same as in stance phase)
        alpha = alpha0 + alphadot0 * t
        alpha_dot = alphadot0

        return x, y, z, xdot, ydot, zdot, alpha, alpha_dot
