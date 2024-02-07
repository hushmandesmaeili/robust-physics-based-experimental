from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from preview_control_params import PreviewParams, PreviewControl, TypeOfPhases
from state import State
from model_control_params import ModelControlParams
from preview_locomotion import PreviewLocomotion
from preview_optimization import PreviewOptimization
from state_trajectory import PiecewisePolynomial, PiecewisePolynomialSegment, StateTrajectory
import cma
import numpy as np
import sympy as sp
from sympy import lambdify
import time

# Define initial state, s_0
c_0 = [0, 0, 1.7]
c_dot_0 = [1.5, 0, 1.5]
s_0 = State(c_0, c_dot_0, 0, 0, 0, 0)

T_step = 0.6  # Desired step duration
d_step = 0.75  # Desired step length
d_direction = np.array([1,0,0])  # Desired direction of travel
alpha_d = 0  # Desired pelvis heading
h_d = 0  # Desired COM height
L_r = 1.0  # Nominal leg length

w_steptime = 10
w_stepdist = 0
w_heading = 0
w_com = 0
w_accel = 0.0
w_leg = 10
w_hip = 10

preview_locomotion = PreviewLocomotion()

def objective_function(U):
    # # Convert U to PreviewControl object
    preview_control = convert_U_to_preview_control(U)

    # # Generate the preview
    start_time = time.time()  # Capture start time

    preview_schedule = preview_locomotion.generate_multi_phase_preview(s_0, preview_control)

    end_time = time.time()  # Capture end time
    duration = end_time - start_time  # Calculate the duration

    print(f"The generate_multi_phase_preview() function took {duration} seconds to complete.")

    # Step duration objective
    T_A = preview_control.params[0].duration + preview_control.params[1].duration + preview_control.params[2].duration
    T_B = preview_control.params[3].duration + preview_control.params[4].duration + preview_control.params[5].duration
    g_steptime = (T_A - T_step)**2 + (T_B - T_step)**2


    # Step length and direction objective
    c_A_end = np.array([preview_schedule.x_segments[2].final_value, preview_schedule.y_segments[2].final_value, preview_schedule.z_segments[2].final_value])
    c_B_end = np.array([preview_schedule.x_segments[5].final_value, preview_schedule.y_segments[5].final_value, preview_schedule.z_segments[5].final_value])
    d_A = np.dot((c_A_end - np.array(c_0)), d_direction)
    d_B = np.dot((c_B_end - np.array(c_A_end)), d_direction)
    g_stepdist = (d_A - d_step)**2 + (d_B - d_step)**2


    # Pelvis heading objective
    g_heading = (preview_schedule.alpha_segments[5].final_value - alpha_d)**2


    # COM height objective
    h_apex_func = preview_schedule.z_segments[2].polynomial # function of time
    dh_dt = sp.diff(h_apex_func, 't')  # Differentiate the polynomial to find the vertical velocity
    critical_points = sp.solveset(dh_dt, 't', domain=sp.S.Reals) # Solve for when the velocity is zero (apex of the flight)

    # Evaluate the polynomial at the critical points to find the apex height
    # and determine the maximum (if multiple critical points)
    h_apex_values = [h_apex_func.subs('t', cp) for cp in critical_points]
    h_apex = max(h_apex_values)

    h_start, h_end = preview_schedule.z_segments[2].initial_value, preview_schedule.z_segments[2].final_value # Calculate using dynamics
    h_epsilon = max(h_apex - h_start, h_apex - h_end)
    g_com = (h_epsilon - h_d)**2


    # # COM acceleration objective
    # g_accel_x_total = sum(integrate_acceleration(segment) for segment in preview_schedule.x_segments)
    # g_accel_y_total = sum(integrate_acceleration(segment) for segment in preview_schedule.y_segments)
    # g_accel_z_total = sum(integrate_acceleration(segment) for segment in preview_schedule.z_segments)

    # # Total COM acceleration modeling objective for all three dimensions
    # g_accel_total = g_accel_x_total + g_accel_y_total + g_accel_z_total
    
    start_time_g_accel = time.time()  # Capture start time for g_accel_total

    g_accel_total = compute_acceleration_concurrently(preview_schedule)

    end_time_g_accel = time.time()  # Capture end time for g_accel_total
    duration_g_accel = end_time_g_accel - start_time_g_accel  # Calculate the duration for g_accel_total
    print(f"The compute_acceleration_concurrently() function took {duration_g_accel} seconds to complete.")

    # Leg length objective
    # TODO: Implement leg length objective

    objective = w_steptime * g_steptime + w_stepdist * g_stepdist + w_heading * g_heading #+ w_com * g_com + w_accel * g_accel_total 
    print(f"Total cost is {objective}.")

    end_time = time.time()  # Capture end time
    duration = end_time - start_time  # Calculate the duration

    print(f"The objective function took {duration} seconds to complete.")

    return objective


def compute_total_acceleration(segments):
    # Function to compute total acceleration for a list of segments
    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(integrate_acceleration, segment) for segment in segments]
        results = [future.result() for future in as_completed(futures)]
    return sum(results)

# Function to concurrently compute the acceleration for x, y, z segments
def compute_acceleration_concurrently(preview_schedule):
    with ProcessPoolExecutor() as executor:
        # Create futures for each dimension
        future_x = executor.submit(compute_total_acceleration, preview_schedule.x_segments)
        future_y = executor.submit(compute_total_acceleration, preview_schedule.y_segments)
        future_z = executor.submit(compute_total_acceleration, preview_schedule.z_segments)
        
        # Wait for all futures to complete and sum their results
        g_accel_x_total = future_x.result()
        g_accel_y_total = future_y.result()
        g_accel_z_total = future_z.result()

    # Sum the totals from each dimension
    g_accel_total = g_accel_x_total + g_accel_y_total + g_accel_z_total
    return g_accel_total


# Example function to calculate the integral of squared acceleration for a segment
def integrate_acceleration(segment):
    t = sp.symbols('t')
    # Assuming segment.polynomial is a sympy expression
    acceleration = sp.diff(segment.polynomial, t, t)  # Second derivative for acceleration
    squared_acceleration = acceleration
    integral_result = sp.integrate(squared_acceleration, (t, segment.start_time, segment.end_time))
    return integral_result


def convert_U_to_preview_control(U):
    preview_control = PreviewControl()

    preview_param_ds_1 = PreviewParams()
    preview_param_ds_1.phase_type = TypeOfPhases.STANCE
    preview_param_ds_1.duration = U[0]
    preview_param_ds_1.p_0 = [U[1], U[2]]
    preview_param_ds_1.p_T = [U[3], U[4]]
    preview_param_ds_1.r_0 = U[5]
    preview_param_ds_1.r_T = U[6]
    preview_param_ds_1.alpha_ddot = U[7]

    preview_param_ss_1 = PreviewParams()
    preview_param_ss_1.phase_type = TypeOfPhases.STANCE
    preview_param_ss_1.duration = U[8]
    preview_param_ss_1.p_0 = [U[3], U[4]]
    preview_param_ss_1.p_T = [U[9], U[10]]
    preview_param_ss_1.r_0 = U[6]
    preview_param_ss_1.r_T = U[11]
    preview_param_ss_1.alpha_ddot = U[12]

    preview_param_f_1 = PreviewParams()
    preview_param_f_1.phase_type = TypeOfPhases.FLIGHT
    preview_param_f_1.duration = U[13]

    preview_param_ds_2 = PreviewParams()
    preview_param_ds_2.phase_type = TypeOfPhases.STANCE
    preview_param_ds_2.duration = U[14]
    preview_param_ds_2.p_0 = [U[15], U[16]]
    preview_param_ds_2.p_T = [U[17], U[18]]
    preview_param_ds_2.r_0 = U[19]
    preview_param_ds_2.r_T = U[20]
    preview_param_ds_2.alpha_ddot = U[21]

    preview_param_ss_2 = PreviewParams()
    preview_param_ss_2.phase_type = TypeOfPhases.STANCE
    preview_param_ss_2.duration = U[22]
    preview_param_ss_2.p_0 = [U[17], U[18]]
    preview_param_ss_2.p_T = [U[23], U[24]]
    preview_param_ss_2.r_0 = U[20]
    preview_param_ss_2.r_T = U[25]
    preview_param_ss_2.alpha_ddot = U[26]

    preview_param_f_2 = PreviewParams()
    preview_param_f_2.phase_type = TypeOfPhases.FLIGHT
    preview_param_f_2.duration = U[27]

    preview_control.params = [preview_param_ds_1, preview_param_ss_1, preview_param_f_1, preview_param_ds_2, preview_param_ss_2, preview_param_f_2]
    preview_control.footplant = [U[28], U[29]]

    return preview_control



def main():
  
    T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1 = 0.3, 0.3, 0, 0.70, 0, 1.3, 1.3, 0
    T_2, p_T_2_x, p_T_2_y, r_T_2, alpha_ddot_2 = 0.3, 0.9, 0, 1.3, 0
    T_3 = 0.3
    T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, alpha_ddot_4 = 0.3, 1.4, 0, 1.8, 0, 1.3, 1.3, 0
    T_5, p_T_5_x, p_T_5_y, r_T_5, alpha_ddot_5 = 0.3, 2.0, 0, 1.3, 0
    T_6 = 0.3
    u_swing_x, u_swing_y = 1.4, 0

    # 23 free parameters, some with x,y components
    U_initial = np.array([T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1, 
                          T_2, p_T_2_x, p_T_2_y, r_T_2, alpha_ddot_2, 
                          T_3, 
                          T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, alpha_ddot_4, 
                          T_5, p_T_5_x, p_T_5_y, r_T_5, alpha_ddot_5, 
                          T_6, 
                          u_swing_x, u_swing_y])

    # total_cost = objective_function(U_initial)

    # print("Total cost: ", total_cost)

    # U_initial = np.zeros(30)  # 23 free parameters, some with x,y components

    # Initial standard deviation
    sigma = 0.5

    # Set up the optimization options, e.g., maximum number of iterations, population size.
    options = {
    'maxfevals': 5,
    'popsize': 15,
    'verb_disp': 1,  # Controls frequency of displayed information (1 for every iteration)
    'verb_log': 1,   # Controls logging frequency to files (1 for every iteration)
    'verb_time': False  # Can be set to True to display timing information
    }   

    # Run the CMA optimization
    es = cma.CMAEvolutionStrategy(U_initial, sigma, options)
    U_star = es.optimize(objective_function).result.xbest
    print("U_star: ", U_star)

if __name__ == "__main__":
    main()