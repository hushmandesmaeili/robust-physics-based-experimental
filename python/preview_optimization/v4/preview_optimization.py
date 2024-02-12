import numpy as np
from libcmaes_api import lcmaes_interface as lci
from preview_locomotion import PreviewControl, PreviewLocomotion, PreviewParams, TypeOfPhases
from state import ReducedBodyState, ReducedBodyTrajectory

class UserParameters:
    def __init__(self, T_step=None, d_step=None, d_direction=None, heading_d=None, h_d=None, L_r=None, 
                 w_steptime=None, w_stepdist=None, w_heading=None, w_com=None, w_accel=None, w_leg=None, w_hip=None):
        # User specified settings for goal objectives
        self._T_step = T_step
        self._d_step = d_step
        self._d_direction = np.array(d_direction)
        self._heading_d = heading_d
        self._h_d = h_d
        self._L_r = L_r

        # User specified settings for weighting factors
        self._w_steptime = w_steptime
        self._w_stepdist = w_stepdist
        self._w_heading = w_heading
        self._w_com = w_com
        self._w_accel = w_accel
        self._w_leg = w_leg
        self._w_hip = w_hip


class PreviewOptimization:
    def __init__(self, preview_locomotion: PreviewLocomotion, user_parameters: UserParameters=None):
        self.preview_locomotion = preview_locomotion
        self._initial_guess = self.convert_array_to_preview_control(np.zeros(self.get_control_dimension()))

        # Initialize user-specified parameters
        self.update_parameters(user_parameters)


    def update_parameters(self, user_parameters: UserParameters):
        if user_parameters.T_step is not None:
            self._T_step = user_parameters.T_step
        if user_parameters.d_step is not None:
            self._d_step = user_parameters.d_step
        if user_parameters.d_direction is not None:
            self._d_direction = user_parameters.d_direction
        if user_parameters.heading_d is not None:
            self._heading_d = user_parameters.heading_d
        if user_parameters.h_d is not None:
            self._h_d = user_parameters.h_d
        if user_parameters.L_r is not None:
            self._L_r = user_parameters.L_r

        if user_parameters.w_steptime is not None:
            self._w_steptime = user_parameters.w_steptime
        if user_parameters.w_stepdist is not None:
            self._w_stepdist = user_parameters.w_stepdist
        if user_parameters.w_heading is not None:
            self._w_heading = user_parameters.w_heading
        if user_parameters.w_com is not None:
            self._w_com = user_parameters.w_com
        if user_parameters.w_accel is not None:
            self._w_accel = user_parameters.w_accel
        if user_parameters.w_leg is not None:
            self._w_leg = user_parameters.w_leg
        if user_parameters.w_hip is not None:
            self._w_hip = user_parameters.w_hip

    def update_initial_guess(self, new_initial_guess: PreviewControl):
        self._initial_guess = new_initial_guess
        
    def get_control_dimension(self):
        return 30

    def run_optimization(self):
        # Define the optimization problem
        init_guess = self.convert_preview_control_to_array(self._initial_guess)
        sigma0 = 0.1
        dimension = self.get_control_dimension()
        # Run optimization via libcmaes_interface (lci)
        res = lci.pcmaes(lci.to_fitfunc(self.objective_func),
                     lci.to_params(init_guess, sigma0,
                                   str_algo=b'cmaes',
                                   lbounds=[-5] * dimension, ubounds=[5] * dimension,
                                   restarts=2))
        
        optimal_U_processed = self.process_optimization_result(res)

        # Update initial guess for next optimization
        self.update_initial_guess(optimal_U_processed)

        return optimal_U_processed

    def process_optimization_result(self, result):
        # Process the optimization result
        # Convert the optimized variables back to control parameters and update the PreviewLocomotion as needed
        optimal_U, objective_value = lci.get_best_candidate(result)

        return self.convert_array_to_preview_control(optimal_U)


    def step_duration_objective(self, U):
        # TODO: Implement the objective function for step duration, 
        #       using any set of control parameters U, e.g. U[0], U[1], U[3], U[4]
        #       or U[1], U[2], U[4], U[5]
        T_A = U[0] + U[1] + U[2]
        T_B = U[3] + U[4] + U[5]
        return (T_A - self._T_step)**2 + (T_B - self._T_step)**2
    
    def step_length_direction_objective(self, U):
        c_A_end = ...  # Calculate using dynamics
        c_B_end = ...
        c_0 = np.array([0, 0, 0])  # Adjust as necessary
        d_A = np.dot((c_A_end - c_0), self._d_direction)
        d_B = np.dot((c_B_end - c_0), self._d_direction)
        return (d_A - self._d_step)**2 + (d_B - self._d_step)**2
    
    def pelvis_heading_objective(self, U):
        alpha_B = ...  # Calculate using dynamics
        return (alpha_B - self._heading_d)**2
    
    def com_height_objective(self, U):
        h_start, h_end, h_apex = ..., ..., ...  # Calculate using dynamics
        h_epsilon = max(h_apex - h_start, h_apex - h_end)
        return (h_epsilon - self._h_d)**2
    
    def com_acceleration_objective(self, U):
        accel_integral = ...  # Implement based on dynamics
        return accel_integral
    
    def leg_length_objective(self, U):
        leg_variation_integral = ...  # Implement based on dynamics
        return leg_variation_integral
    
    def foot_position_objective(self, U):
        hip_ankle_distance_integral = ...  # Implement based on dynamics
        return hip_ankle_distance_integral
    
    def objective_func(self, U):
        # Combine all objectives, with weighting factors
        total = 0
        total += self._w_steptime * self.step_duration_objective(U)
        total += self._w_stepdist * self.step_length_direction_objective(U)
        total += self._w_heading * self.pelvis_heading_objective(U)
        total += self._w_com * self.com_height_objective(U)
        total += self._w_accel * self.com_acceleration_objective(U)
        total += self._w_leg * self.leg_length_objective(U)
        total += self._w_hip * self.foot_position_objective(U)
        
        return total

    def convert_preview_control_to_array(self, preview_control: PreviewControl):
        # This method should extract the parameters from a PreviewControl object and return them as a numpy array.
        arr = np.zeros(self.get_control_dimension())
        arr[0] = preview_control.params[0].duration
        arr[1] = preview_control.params[0].p_0[0]
        arr[2] = preview_control.params[0].p_0[1]
        arr[3] = preview_control.params[0].p_T[0]
        arr[4] = preview_control.params[0].p_T[1]
        arr[5] = preview_control.params[0].r_0
        arr[6] = preview_control.params[0].r_T
        arr[7] = preview_control.params[0].angular_accel
        arr[8] = preview_control.params[1].duration
        arr[9] = preview_control.params[1].p_T[0]
        arr[10] = preview_control.params[1].p_T[1]
        arr[11] = preview_control.params[1].r_T
        arr[12] = preview_control.params[1].angular_accel
        arr[13] = preview_control.params[2].duration
        arr[14] = preview_control.params[3].duration
        arr[15] = preview_control.params[3].p_0[0]
        arr[16] = preview_control.params[3].p_0[1]
        arr[17] = preview_control.params[3].p_T[0]
        arr[18] = preview_control.params[3].p_T[1]
        arr[19] = preview_control.params[3].r_0
        arr[20] = preview_control.params[3].r_T
        arr[21] = preview_control.params[3].angular_accel
        arr[22] = preview_control.params[4].duration
        arr[23] = preview_control.params[4].p_T[0]
        arr[24] = preview_control.params[4].p_T[1]
        arr[25] = preview_control.params[4].r_T
        arr[26] = preview_control.params[4].angular_accel
        arr[27] = preview_control.params[5].duration
        arr[28] = preview_control.footplant[0]
        arr[29] = preview_control.footplant[1]
        
        return arr

    def convert_array_to_preview_control(self, arr):
        # Convert the optimized numpy array back into a PreviewControl object.
        # This method should be the inverse of convert_preview_control_to_array.
        # Implement the actual conversion logic based on your PreviewControl structure.
        if len(arr) != self.get_control_dimension():
            raise ValueError("The length of the input array does not match the control dimension.")
        
        preview_control = PreviewControl()
        
        preview_param_ds_1 = PreviewParams()
        preview_param_ds_1.phase_type = TypeOfPhases.STANCE
        preview_param_ds_1.duration = arr[0]
        preview_param_ds_1.p_0 = np.array([arr[1], arr[2]])
        preview_param_ds_1.p_T = np.array([arr[3], arr[4]])
        preview_param_ds_1.r_0 = arr[5]
        preview_param_ds_1.r_T = arr[6]
        preview_param_ds_1.angular_accel = arr[7]

        preview_param_ss_1 = PreviewParams()
        preview_param_ss_1.phase_type = TypeOfPhases.STANCE
        preview_param_ss_1.duration = arr[8]
        preview_param_ss_1.p_0 = np.array([arr[3], arr[4]])
        preview_param_ss_1.p_T = np.array([arr[9], arr[10]])
        preview_param_ss_1.r_0 = arr[6]
        preview_param_ss_1.r_T = arr[11]
        preview_param_ss_1.angular_accel = arr[12]

        preview_param_f_1 = PreviewParams()
        preview_param_f_1.phase_type = TypeOfPhases.FLIGHT
        preview_param_f_1.duration = arr[13]

        preview_param_ds_2 = PreviewParams()
        preview_param_ds_2.phase_type = TypeOfPhases.STANCE
        preview_param_ds_2.duration = arr[14]
        preview_param_ds_2.p_0 = np.array([arr[15], arr[16]])
        preview_param_ds_2.p_T = np.array([arr[17], arr[18]])
        preview_param_ds_2.r_0 = arr[19]
        preview_param_ds_2.r_T = arr[20]
        preview_param_ds_2.angular_accel = arr[21]

        preview_param_ss_2 = PreviewParams()
        preview_param_ss_2.phase_type = TypeOfPhases.STANCE
        preview_param_ss_2.duration = arr[22]
        preview_param_ss_2.p_0 = np.array([arr[17], arr[18]])
        preview_param_ss_2.p_T = np.array([arr[23], arr[24]])
        preview_param_ss_2.r_0 = arr[20]
        preview_param_ss_2.r_T = arr[25]
        preview_param_ss_2.angular_accel = arr[26]

        preview_param_f_2 = PreviewParams()
        preview_param_f_2.phase_type = TypeOfPhases.FLIGHT
        preview_param_f_2.duration = arr[27]

        preview_control.params = [preview_param_ds_1, preview_param_ss_1, preview_param_f_1, preview_param_ds_2, preview_param_ss_2, preview_param_f_2]
        preview_control.footplant = np.array([arr[28], arr[29]])

        return preview_control