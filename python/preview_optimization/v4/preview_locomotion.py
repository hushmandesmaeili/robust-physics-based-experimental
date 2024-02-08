import copy
from enum import Enum
from typing import List
from stance_dynamics import StanceControlParams, StanceDynamics
from flight_dynamics import FlightControlParams, FlightDynamics
import numpy as np

from state import ReducedBodyTrajectory, ReducedBodyState

class TypeOfPhases(Enum):
    STANCE = 1
    FLIGHT = 2

class PreviewParams:
    def __init__(self, duration: float=None, p_0=None, p_T=None, 
                 r_0=None, r_T=None, angular_accel=None, phase_type=None, _id: int = 0):
        # self.id = _id
        self.duration = duration
        self.p_0 = p_0
        self.p_T = p_T
        self.r_0 = r_0
        self.r_T = r_T
        self.angular_accel = angular_accel
        self.phase_type = phase_type  # This will be an instance of PreviewPhase

class PreviewControl:
    def __init__(self, _params: List[PreviewParams] = []):
        self.params = _params
        self.footplant = None

    def getTotalDuration(self) -> float:
        return sum(param.duration for param in self.params)
    
    def setFootplant(self, footplant):
        self.footplant = footplant
        

class PreviewLocomotion:
    def __init__(self):
        # Actual reduced-body state
        self._actual_state = None

        # Placeholder for whole-body dynamics
        # Placeholder for whole-body kinematics
        # Placeholder for floating-based information

        # Label to indicate whether robot information was set
        self._robot_model_set = False

        # Sample time of the preview trajectory
        self._sample_time = 0.01
        
        self._gravity = 9.81
        self._mass = 10.0

        self._num_feet = 2

        self._stance_dynamics = StanceDynamics()
        self._flight_dynamics = FlightDynamics()

        self._step_height = 0.05

    def set_sample_time(self, sample_time: float):
        self._sample_time = sample_time

    def set_step_height(self, step_height: float):
        self._step_height = step_height

    def multiphase_preview(self, trajectory: ReducedBodyTrajectory, state: ReducedBodyState,
                           preview_control: PreviewControl):
        self._actual_state = state

        trajectory.clear()

        # Computing multi-phase preview
        self._initial_state = state
        num_phases = len(preview_control.params)
        for i in range(2):
            # Debugging phases by clearing trajectory
            # trajectory.clear()

            phase_traj = ReducedBodyTrajectory()

            preview_params = preview_control.params[i]
            if preview_params.phase_type == TypeOfPhases.STANCE:
                self.stance_preview(phase_traj, self._initial_state, preview_params)
            elif preview_params.phase_type == TypeOfPhases.FLIGHT:
                self.flight_preview(phase_traj, self._initial_state, preview_params)
            else:
                raise ValueError("Invalid phase type")
            
            # Append the phase trajectory to the whole trajectory
            trajectory.states.extend(phase_traj.states)

            # Sanity action: defining the actual state if there isn't a trajectory
            if len(trajectory.states) == 0:
                trajectory.add_state(state)

            # Updating the initial state for the next phase
            self._initial_state = trajectory.states[-1]

    
    def stance_preview(self, trajectory: ReducedBodyTrajectory, state: ReducedBodyState,
                       preview_params: PreviewParams):
        # Checking preview duration
        if preview_params.duration < self._sample_time:
            # raise ValueError("Invalid preview duration")
            return
        
        # Adding support region, constant throughout the phase
        current_state = copy.deepcopy(state)
        # for i in range(self._num_feet):

        # Initializing the stance dynamics
        model_params = StanceControlParams(preview_params.duration, preview_params.p_0, preview_params.p_T,
                                           preview_params.r_0, preview_params.r_T, preview_params.angular_accel)
    
        self._stance_dynamics.init_response(current_state, model_params)
        num_samples = int(preview_params.duration / self._sample_time)
    
        trajectory.states = [None] * (num_samples)

        time = 0
        for i in range(num_samples):
            time = self._sample_time * (i + 1)
            current_state.time = state.time + time

            self._stance_dynamics.compute_response(current_state, current_state.time)

            trajectory.states[i] = copy.deepcopy(current_state)

    def flight_preview(self, trajectory: ReducedBodyTrajectory, state: ReducedBodyState,
                       preview_params: PreviewParams):
        # Checking preview duration
        if preview_params.duration < self._sample_time:
            # raise ValueError("Invalid preview duration")
            return
        
        # Adding support region, constant throughout the phase
        current_state = copy.deepcopy(state)
        # for i in range(self._num_feet):

        # Initializing the stance dynamics
        model_params = FlightControlParams(preview_params.duration)
    
        self._flight_dynamics.init_response(current_state, model_params)
        num_samples = int(preview_params.duration / self._sample_time)
    
        trajectory.states = [None] * (num_samples)

        time = 0
        for i in range(num_samples):
            time = self._sample_time * (i + 1)
            current_state.time = state.time + time

            self._flight_dynamics.compute_response(current_state, current_state.time)

            trajectory.states[i] = copy.deepcopy(current_state)