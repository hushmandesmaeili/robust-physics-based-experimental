import copy
from math import cos, exp, sin, sqrt
from state import ReducedBodyState
import numpy as np

# Define StanceControlParams class
class StanceControlParams:
    def __init__(self, duration=None, p_0=None, p_T=None, r_0=None, r_T=None, angular_accel=None):
        """
        Initialize the StanceControlParams object.

        Parameters:
        T_duration (float): The duration of the stance phase.
        p_0 (float): The initial position of the COP.
        p_T (float): The final position of the COP.
        r_0 (float): The initial rest length of the model.
        r_T (float): The final rest length of the model.
        angular_accel (float): The angular acceleration of the robot.
        """
        self.duration = duration
        self.p_0 = p_0
        self.p_T = p_T
        self.r_0 = r_0
        self.r_T = r_T
        self.angular_accel = angular_accel

# Define StanceDynamics class
class StanceDynamics:
    def __init__(self, g=9.81, k=1, m=10):
        self.g = g
        self.k = k
        self.m = m

        self._initial_state = None
        self._control_params = None

        self._height = None
        self._alpha = None
        self._horz_proj = None
        self._horz_disp = None
        self._beta_1 = None
        self._beta_2 = None
        self._cop_T = None

        self._omega = None
        self._d_1 = None
        self._d_2 = None
        self._r_T = None

    def init_response(self, initial_state: ReducedBodyState, control_params: StanceControlParams):
        self._initial_state = copy.deepcopy(initial_state)
        self._control_params = copy.deepcopy(control_params)

        # Computing constants and expressions for horizontal motion
        self._height = initial_state.com_pos[2]
        self._alpha = sqrt(self.g / self._height)        # omega in other implementations
        gamma = 2 * self._alpha * control_params.duration * self._control_params.duration   # alpha in other implementation
        self._horz_proj = self._initial_state.com_pos[:2] - self._initial_state.cop[:2]     # x_0 - p_0, for x and y
        self._horz_disp = self._initial_state.com_vel[:2] * control_params.duration     # x_dot_0 * T, for x and y
        self._beta_1 = (self._horz_proj / 2) + (self._horz_disp - (self._control_params.p_T - self._control_params.p_0)) / (gamma)
        self._beta_2 = (self._horz_proj / 2) - (self._horz_disp - (self._control_params.p_T - self._control_params.p_0)) / (gamma)
        self._cop_T = (control_params.p_T - control_params.p_0) / control_params.duration

        # Computing constants and expressions for vertical motion
        self._omega = sqrt(self.k / self.m)
        self._d_1 = self._initial_state.com_pos[2] - self._initial_state.rest_length + self.g / self._omega**2
        self._d_2 = self._initial_state.com_vel[2] / self._omega - (self._control_params.r_T - self._control_params.r_0) / (self._control_params.duration * self._omega)
        self._r_T = (self._control_params.r_T - self._control_params.r_0) / self._control_params.duration


    def compute_response(self, state: ReducedBodyState, time):
        # Computing the delta time w.r.t. the initial time
        dt = time - self._initial_state.time
        state.time = time

        # Computing CoP motion
        delta_cop = dt * self._cop_T
        state.cop = self._initial_state.cop + delta_cop

        # Computing the horizontal motion of the CoM
        beta_1_exp = self._beta_1 * exp(self._alpha * dt)
        beta_2_exp = self._beta_2 * exp(-self._alpha * dt)
        state.com_pos[:2] = beta_1_exp + beta_2_exp + (self._cop_T * dt + self._initial_state.cop)  # x(t) = beta_1_exp + beta_2_exp + p_x(t)
        state.com_vel[:2] = self._alpha * (beta_1_exp - beta_2_exp) + self._cop_T  # x_dot(t) = alpha * (beta_1_exp - beta_2_exp) + p_x_dot(t)
        state.com_acc[:2] = self._alpha**2 * (beta_1_exp + beta_2_exp)  # x_ddot(t) = alpha^2 * (beta_1_exp + beta_2_exp)

        # Computing the vertical motion of the CoM
        d_1_exp = self._d_1 * cos(self._omega * dt)
        d_2_exp = self._d_2 * sin(self._omega * dt)
        state.com_pos[2] = d_1_exp + d_2_exp + (self._r_T * dt + self._initial_state.rest_length) - self.g / self._omega**2  # z(t) = d_1_exp + d_2_exp + r(t) - g / omega^2
        state.com_vel[2] = -self._omega * self._d_1 * sin(self._omega * dt) + self._omega * self._d_2 * cos(self._omega * dt) + self._r_T
        state.com_acc[2] = -self._omega**2 * d_1_exp - self._omega**2 * d_2_exp

        # Computing the orientation motion of the robot
        state.angular_pos = self._initial_state.angular_pos +  self._initial_state.angular_vel * dt + 0.5 * self._control_params.angular_accel * dt**2
        state.angular_vel = self._initial_state.angular_vel + self._control_params.angular_accel * dt
        state.angular_acc = self._control_params.angular_accel

