# Define FlightControlParams class
import numpy as np
from state import ReducedBodyState


class FlightControlParams:
    def __init__(self, duration=None):
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

# Define FlightDynamics class
class FlightDynamics:
    def __init__(self, g=9.81, k=1, m=10):
        self.g = g
        self.k = k
        self.m = m

        self._initial_state = None
        self._control_params = None

        self._c_accel = np.array([0, 0, self.g])

    def init_response(self, initial_state: ReducedBodyState, control_params: FlightControlParams):
        self._initial_state = initial_state
        self._control_params = control_params

    def compute_response(self, state: ReducedBodyState, time: float):
        # Computing the delta time w.r.t. the initial time
        dt = time - self._initial_state.time
        state.time = time

        # Computing the linear motion of the CoM
        state.com_pos = self._initial_state.com_pos + self._initial_state.com_vel * dt - 0.5 * self._c_accel * dt**2
        state.com_vel = self._initial_state.com_vel - self._c_accel * dt
        state.com_acc = -self._c_accel

        # Computing the rotational motion of the CoM
        state.angular_pos = self._initial_state.angular_pos + self._initial_state.angular_vel * dt
        state.angular_vel = self._initial_state.angular_vel
        state.angular_acc = 0