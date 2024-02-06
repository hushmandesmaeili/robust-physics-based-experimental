import sympy as sp
from state import State
from model_control_params import ModelControlParams

# Define StanceDynamics class
class StanceDynamics:
    def __init__(self, g=9.81, k=1, m=10):
        self.g = g
        self.k = k
        self.m = m

    def simulate(self, initial_state: State, control_input: ModelControlParams):
        t = sp.symbols('t')

        c_0, c_dot_0 = initial_state.c, initial_state.c_dot
        alpha0, alpha_dot0, omega_R, omega_L = initial_state.alpha, initial_state.alpha_dot, initial_state.omega_R, initial_state.omega_L
        x0, y0, z0 = c_0
        xdot0, ydot0, zdot0 = c_dot_0

        T_total, p0, pT, r0, rT, alpha_ddot = control_input.T_duration, control_input.p_0, control_input.p_T, control_input.r_0, control_input.r_T, control_input.alpha_ddot

        p0x, p0y = p0
        pTx, pTy = pT

        g, k, m = self.g, self.k, self.m
        h = z0
        
        # Constants
        alpha_const = sp.sqrt(g / h)

        omega = sp.sqrt(k / m)

        # Beta calculations
        betax1 = ((x0 - p0x) / 2) + ((xdot0 * T_total - (pTx - p0x)) / (2 * alpha_const * T_total))
        betax2 = ((x0 - p0x) / 2) - ((xdot0 * T_total - (pTx - p0x)) / (2 * alpha_const * T_total))
        betay1 = ((y0 - p0y) / 2) + ((ydot0 * T_total - (pTy - p0y)) / (2 * alpha_const * T_total))
        betay2 = ((y0 - p0y) / 2) - ((ydot0 * T_total - (pTy - p0y)) / (2 * alpha_const * T_total))

        # Horizontal motion equations
        px = (t / T_total) * (pTx - p0x) + p0x
        py = (t / T_total) * (pTy - p0y) + p0y
        x = betax1 * sp.exp(alpha_const * t) + betax2 * sp.exp(-alpha_const * t) + px
        y = betay1 * sp.exp(alpha_const * t) + betay2 * sp.exp(-alpha_const * t) + py

        xdot = sp.diff(x, t)
        ydot = sp.diff(y, t)

        # Vertical motion equation
        d1 = z0 - r0 + g / omega**2
        d2 = zdot0 / omega - (rT - r0) / (T_total * omega)
        r = (t / T_total) * (rT - r0) + r0
        z = d1 * sp.cos(omega * t) + d2 * sp.sin(omega * t) + r - g / omega**2
        
        zdot = sp.diff(z, t)

        # Heading equation
        alpha = alpha0 + alpha_dot0 * t + 0.5 * alpha_ddot * t**2
        
        alpha_dot = sp.diff(alpha, t)

        return x, y, z, xdot, ydot, zdot, alpha, alpha_dot