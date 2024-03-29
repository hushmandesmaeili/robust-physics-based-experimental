import numpy as np
from com_trajectory_plotter import COMTrajectoryPlotter
from preview_optimization import PreviewOptimization, UserParameters
from state import ReducedBodyState
from preview_locomotion import PreviewLocomotion, PreviewControl, PreviewParams, TypeOfPhases

def create_initial_preview_control():
    # Define initial parameters for the PreviewControl object
    T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, angular_accel_1 = 0.3, 0.3, 0, 0.70, 0, 20.3, 500.3, 0
    T_2, p_T_2_x, p_T_2_y, r_T_2, angular_accel_2 = 0.3, 0.9, 0, 20.3, 0
    T_3 = 0.0
    T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, angular_accel_4 = 0.3, 1.4, 0, 1.8, 0, 20.3, 20.3, 0
    T_5, p_T_5_x, p_T_5_y, r_T_5, angular_accel_5 = 0.3, 2.0, 0, 10.3, 0
    T_6 = 0.0
    u_swing_x, u_swing_y = 1.4, 0

    # 23 free parameters, some with x,y components
    U = np.array([T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, angular_accel_1, 
                          T_2, p_T_2_x, p_T_2_y, r_T_2, angular_accel_2, 
                          T_3, 
                          T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, angular_accel_4, 
                          T_5, p_T_5_x, p_T_5_y, r_T_5, angular_accel_5, 
                          T_6, 
                          u_swing_x, u_swing_y])
    
    preview_control = PreviewControl()

    preview_param_ds_1 = PreviewParams()
    preview_param_ds_1.phase_type = TypeOfPhases.STANCE
    preview_param_ds_1.duration = U[0]
    preview_param_ds_1.p_0 = np.array([U[1], U[2]])
    preview_param_ds_1.p_T = np.array([U[3], U[4]])
    preview_param_ds_1.r_0 = U[5]
    preview_param_ds_1.r_T = U[6]
    preview_param_ds_1.angular_accel = U[7]

    preview_param_ss_1 = PreviewParams()
    preview_param_ss_1.phase_type = TypeOfPhases.STANCE
    preview_param_ss_1.duration = U[8]
    preview_param_ss_1.p_0 = np.array([U[3], U[4]])
    preview_param_ss_1.p_T = np.array([U[9], U[10]])
    preview_param_ss_1.r_0 = U[6]
    preview_param_ss_1.r_T = U[11]
    preview_param_ss_1.angular_accel = U[12]

    preview_param_f_1 = PreviewParams()
    preview_param_f_1.phase_type = TypeOfPhases.FLIGHT
    preview_param_f_1.duration = U[13]

    preview_param_ds_2 = PreviewParams()
    preview_param_ds_2.phase_type = TypeOfPhases.STANCE
    preview_param_ds_2.duration = U[14]
    preview_param_ds_2.p_0 = np.array([U[15], U[16]])
    preview_param_ds_2.p_T = np.array([U[17], U[18]])
    preview_param_ds_2.r_0 = U[19]
    preview_param_ds_2.r_T = U[20]
    preview_param_ds_2.angular_accel = U[21]

    preview_param_ss_2 = PreviewParams()
    preview_param_ss_2.phase_type = TypeOfPhases.STANCE
    preview_param_ss_2.duration = U[22]
    preview_param_ss_2.p_0 = np.array([U[17], U[18]])
    preview_param_ss_2.p_T = np.array([U[23], U[24]])
    preview_param_ss_2.r_0 = U[20]
    preview_param_ss_2.r_T = U[25]
    preview_param_ss_2.angular_accel = U[26]

    preview_param_f_2 = PreviewParams()
    preview_param_f_2.phase_type = TypeOfPhases.FLIGHT
    preview_param_f_2.duration = U[27]

    preview_control.params = [preview_param_ds_1, preview_param_ss_1, preview_param_f_1, preview_param_ds_2, preview_param_ss_2, preview_param_f_2]
    preview_control.footplant = np.array([U[28], U[29]])

    return preview_control

def main():
    # Define initial state
    c_0 = np.array([0.2, 0, 1.7])
    c_dot_0 = np.array([1.5, 0, 1.5])
    s_0 = ReducedBodyState(com_pos=c_0, com_vel=c_dot_0, angular_pos=0, angular_vel=0, time=0)

    # Define user parameters
    T_step = 0.6
    d_step = 0.75
    d_direction = np.array([1, 0, 0])
    alpha_d = 0
    h_d = 0
    L_r = 1.0

    w_steptime = 10
    w_stepdist = 0
    w_heading = 0
    w_com = 0
    w_accel = 0.0
    w_leg = 10
    w_hip = 10

    user_params = UserParameters(T_step, d_step, d_direction, alpha_d, h_d, L_r, 
                                 w_steptime, w_stepdist, w_heading, w_com, w_accel, w_leg, w_hip)

    # Initialize PreviewControl and PreviewLocomotion
    preview_control_initial = create_initial_preview_control()
    preview_locomotion = PreviewLocomotion()

    # Pass the initial PreviewControl object to PreviewOptimization for optimization
    preview_optimization = PreviewOptimization(preview_locomotion, user_params)
    preview_optimization.update_initial_guess(preview_control_initial)
    optimized_preview_control = preview_optimization.run_optimization()

    # Use the optimized PreviewControl object for locomotion
    trajectory = preview_locomotion.multiphase_preview(s_0, optimized_preview_control)

    # Plot the 3D COM trajectory
    com_trajectory_plotter = COMTrajectoryPlotter(trajectory, optimized_preview_control)
    com_trajectory_plotter.plot_3d_com_trajectory()

if __name__ == "__main__":
    main()
