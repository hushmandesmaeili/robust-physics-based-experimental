from typing import List
from sympy import symbols

from state import State


class StateTrajectory:
    def __init__(self, x, y, z, alpha, xdot, ydot, zdot, alpha_dot):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.xdot = xdot
        self.ydot = ydot
        self.zdot = zdot
        self.alpha_dot = alpha_dot

    def evaluate_trajectory_at_time(self, time):
        """Evaluate the trajectory at a specific time."""
        x = self.x.subs('t', time)
        y = self.y.subs('t', time)
        z = self.z.subs('t', time)
        alpha = self.alpha.subs('t', time)
        x_dot = self.xdot.subs('t', time)
        y_dot = self.ydot.subs('t', time)
        z_dot = self.zdot.subs('t', time)
        alpha_dot = self.alpha_dot.subs('t', time)
        
        c = [x, y, z]
        c_dot = [x_dot, y_dot, z_dot]

        return c, c_dot, alpha, alpha_dot

# class MultiPhaseStateTrajectory:
#     def __init__(self, _trajectories: List[StateTrajectory] = []):
#         self.trajectories = _trajectories

#     def clear(self):
#         self.trajectories.clear()

#     def append(self, phase_trajectory):
#         """Appends a new phase trajectory to the trajectory list."""
#         self.trajectories.append(phase_trajectory)
class PiecewisePolynomialSegment:
    def __init__(self, polynomial, start_time, end_time, initial_value, final_value):
        self.polynomial = polynomial
        self.start_time = start_time
        self.end_time = end_time
        self.initial_value = initial_value
        self.final_value = final_value
    
class PiecewisePolynomial:
    def __init__(self, _x_segs=[], _y_segs=[], _z_segs=[], _alpha_segs=[], _xdot_segs=[], _ydot_segs=[], _zdot_segs=[], _alpha_dot_segs=[]):
        # Initialize dictionaries to hold polynomial segments for each trajectory component
        self.x_segments = _x_segs
        self.y_segments = _y_segs
        self.z_segments = _z_segs
        self.alpha_segments = _alpha_segs
        self.xdot_segments = _xdot_segs
        self.ydot_segments = _ydot_segs
        self.zdot_segments = _zdot_segs
        self.alpha_dot_segments = _alpha_dot_segs

    def add_segment(self, state_trajectory: StateTrajectory, start_time, end_time, initial_state: State, final_state: State):
        """
        Adds a new segment to the piecewise polynomial for each component.
        
        :param state_trajectory: An instance of StateTrajectory containing the polynomial expressions for the phase.
        :param start_time: The start time of the phase.
        :param end_time: The end time of the phase.
        :param initial_state: The initial state at the start of the phase.
        :param final_state: The final state at the end of the phase.
        """
        # Each 'segment' is a tuple (polynomial, start_time, end_time, initial_state.component, final_state.component)
        self.x_segments.append(PiecewisePolynomialSegment(state_trajectory.x, start_time, end_time, initial_state.c[0], final_state.c[0]))
        self.y_segments.append(PiecewisePolynomialSegment(state_trajectory.y, start_time, end_time, initial_state.c[1], final_state.c[1]))
        self.z_segments.append(PiecewisePolynomialSegment(state_trajectory.z, start_time, end_time, initial_state.c[2], final_state.c[2]))
        self.alpha_segments.append(PiecewisePolynomialSegment(state_trajectory.alpha, start_time, end_time, initial_state.alpha, final_state.alpha))
        self.xdot_segments.append(PiecewisePolynomialSegment(state_trajectory.xdot, start_time, end_time, initial_state.c_dot[0], final_state.c_dot[0]))
        self.ydot_segments.append(PiecewisePolynomialSegment(state_trajectory.ydot, start_time, end_time, initial_state.c_dot[1], final_state.c_dot[1]))
        self.zdot_segments.append(PiecewisePolynomialSegment(state_trajectory.zdot, start_time, end_time, initial_state.c_dot[2], final_state.c_dot[2]))
        self.alpha_dot_segments.append(PiecewisePolynomialSegment(state_trajectory.alpha_dot, start_time, end_time, initial_state.alpha_dot, final_state.alpha_dot))

    # def evaluate_at_end(self):
    #     """
    #     Evaluates each component of the piecewise polynomial at the end time of each segment.
        
    #     :return: A State instance with evaluated values.
    #     """
    #     x = self._evaluate_component_at_end(self.x_segments)
    #     y = self._evaluate_component_at_end(self.y_segments)
    #     z = self._evaluate_component_at_end(self.z_segments)
    #     alpha = self._evaluate_component_at_end(self.alpha_segments)
    #     # xdot = self._evaluate_component_at_end(self.xdot_segments)
    #     # ydot = self._evaluate_component_at_end(self.ydot_segments)
    #     # zdot = self._evaluate_component_at_end(self.zdot_segments)
    #     # alpha_dot = self._evaluate_component_at_end(self.alpha_dot_segments)

    #     # return State(x, y, z, alpha, xdot, ydot, zdot, alpha_dot)
    #     return State(x, y, z, alpha)
    
    # def _evaluate_component_at_end(self, segments):
    #     """
    #     Helper method to evaluate a specific component at the end time of each segment.
        
    #     :param segments: The list of segments for a specific component.
    #     :return: The evaluated value at the end time of each segment.
    #     """
    #     values = []
    #     for polynomial, start_time, end_time in segments:
    #         # Assume 'polynomial' has a method 'subs' to substitute a value for the variable
    #         values.append(polynomial.subs('t', end_time))
    #     return values

    # def evaluate_at_time(self, time):
    #     """
    #     Evaluates each component of the piecewise polynomial at a given time.
        
    #     :param time: The time at which to evaluate the polynomial.
    #     :return: A StateTrajectory instance with evaluated values.
    #     """
    #     x = self._evaluate_component_at_time(self.x_segments, time)
    #     y = self._evaluate_component_at_time(self.y_segments, time)
    #     # Repeat for other components...
    #     # Dummy implementations for z, alpha, etc., assuming a similar evaluation process
    #     z = alpha = xdot = ydot = zdot = alpha_dot = 0  # Placeholder for actual evaluations

    #     return StateTrajectory(x, y, z, alpha, xdot, ydot, zdot, alpha_dot)

    # def _evaluate_component_at_time(self, segments, time):
    #     """
    #     Helper method to evaluate a specific component at a given time.
        
    #     :param segments: The list of segments for a specific component.
    #     :param time: The time at which to evaluate.
    #     :return: The evaluated value at the given time.
    #     """
    #     for polynomial, start_time, end_time in segments:
    #         if start_time <= time < end_time:
    #             # Assume 'polynomial' has a method 'evaluate_at' to evaluate it at a specific time
    #             return polynomial.evaluate_at(time)
    #     # If time is not within any segment, handle accordingly (e.g., return 0 or raise an error)
    #     return 0
