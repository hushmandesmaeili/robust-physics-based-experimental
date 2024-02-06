# Define PreviewLocomotion class
from math_utils import MathUtils
from preview_control_params import PreviewControl, PreviewParams, TypeOfPhases
from model_control_params import ModelControlParams
from state import State
from stance_dynamics import StanceDynamics
from flight_dynamics import FlightDynamics
from state_trajectory import PiecewisePolynomial, StateTrajectory

class PreviewLocomotion:
    def generate_stance_preview(self, initial_state, control_input):
        # Generate preview using the stance dynamics model
        dynamics_model = StanceDynamics()
        x, y, z, xdot, ydot, zdot, alpha, alpha_dot = dynamics_model.simulate(initial_state, control_input)

        return StateTrajectory(x, y, z, alpha, xdot, ydot, zdot, alpha_dot)

        # # Convert the preview to a Taylor polynomial
        # taylor_x = MathUtils.taylor_approximation(x, 5, 0) # assuming expansion around t=0
        # taylor_y = MathUtils.taylor_approximation(y, 5, 0)
        # taylor_z = MathUtils.taylor_approximation(z, 5, 0)
        # taylor_xdot = MathUtils.taylor_approximation(xdot, 5, 0)
        # taylor_ydot = MathUtils.taylor_approximation(ydot, 5, 0)
        # taylor_zdot = MathUtils.taylor_approximation(zdot, 5, 0)
        # taylor_alpha = MathUtils.taylor_approximation(alpha, 5, 0)
        # taylor_alpha_dot = MathUtils.taylor_approximation(alpha_dot, 5, 0)

        # # Pack approximated trajectories into a SymbolicTrajectory object
        # approx_trajectory = StateTrajectory(taylor_x, taylor_y, taylor_z, taylor_alpha, taylor_xdot, taylor_ydot, taylor_zdot, taylor_alpha_dot)

        # return approx_trajectory

    def generate_flight_preview(self, initial_state, control_input):
        dynamics_model = FlightDynamics()
        x, y, z, xdot, ydot, zdot, alpha, alpha_dot = dynamics_model.simulate(initial_state, control_input)

        return StateTrajectory(x, y, z, alpha, xdot, ydot, zdot, alpha_dot)

        # Convert the preview to a Taylor polynomial
        # taylor_x = MathUtils.taylor_approximation(x, 5, 0) # assuming expansion around t=0
        # taylor_y = MathUtils.taylor_approximation(y, 5, 0)
        # taylor_z = MathUtils.taylor_approximation(z, 5, 0)
        # taylor_xdot = MathUtils.taylor_approximation(xdot, 5, 0)
        # taylor_ydot = MathUtils.taylor_approximation(ydot, 5, 0)
        # taylor_zdot = MathUtils.taylor_approximation(zdot, 5, 0)
        # taylor_alpha = MathUtils.taylor_approximation(alpha, 5, 0)
        # taylor_alpha_dot = MathUtils.taylor_approximation(alpha_dot, 5, 0)

        # # Pack approximated trajectories into a SymbolicTrajectory object
        # approx_trajectory = StateTrajectory(taylor_x, taylor_y, taylor_z, taylor_alpha, taylor_xdot, taylor_ydot, taylor_zdot, taylor_alpha_dot)

        # return approx_trajectory

    def generate_phase(self, phase_type, initial_state, phase_param):
        # Generate the specified phase based on phase_type
        if phase_type == TypeOfPhases.STANCE:
            model_control_params = ModelControlParams(phase_param.duration, phase_param.p_0, phase_param.p_T, phase_param.r_0, phase_param.r_T, phase_param.alpha_ddot)
            return self.generate_stance_preview(initial_state, model_control_params)
        elif phase_type == TypeOfPhases.FLIGHT:
            model_control_params = ModelControlParams(phase_param.duration)
            model_control_params.T_duration = phase_param.duration
            return self.generate_flight_preview(initial_state, model_control_params)
        else:
            raise ValueError("Unknown phase type")

    # def generate_multi_phase_preview(self, _initial_state: State, control: PreviewControl):

    #     initial_state = _initial_state

    #     num_phases = len(control)
    #     for i in range(num_phases):

    #     return trajectory
        
    def generate_multi_phase_preview(self, _initial_state: State, control: PreviewControl):
        initial_state = _initial_state
        current_time = 0
        piecewise_polynomial = PiecewisePolynomial()

        for phase_param in control.params:
            # Generate phase preview
            phase_preview = self.generate_phase(
                phase_param.phase_type,
                initial_state,
                phase_param
            )

            # Get final state
            c, c_dot, alpha, alpha_dot = phase_preview.evaluate_trajectory_at_time(phase_param.duration)
            final_state = State(c, c_dot, alpha, alpha_dot, 0, 0)

            # Update the piecewise polynomial with the new segment
            piecewise_polynomial.add_segment(phase_preview, current_time, current_time + phase_param.duration, initial_state, final_state)

            # Prepare for the next phase
            initial_state = final_state
            current_time += phase_param.duration

        return piecewise_polynomial
    
    # def generate_symbolic_multiphase_preview(self, _initial_state, control):
    #     initial_state = _initial_state
    #     current_time = 0
    #     piecewise_polynomial = PiecewisePolynomial()

    #     for phase_param in control.params:
    #         # Generate phase preview
    #         phase_preview = self.generate_phase(
    #             phase_param.phase_type,
    #             initial_state,
    #             phase_param
    #         )

    #         # Get final state
    #         c, c_dot, alpha, alpha_dot = phase_preview.evaluate_trajectory_at_time(phase_param.duration)
    #         final_state = State(c, c_dot, alpha, alpha_dot, 0, 0)

    #         # Update the piecewise polynomial with the new segment
    #         piecewise_polynomial.add_segment(phase_preview, current_time, current_time + phase_param.duration, initial_state, final_state)

    #         # Prepare for the next phase
    #         initial_state = final_state
    #         current_time += phase_param.duration

    #     return piecewise_polynomial
    
