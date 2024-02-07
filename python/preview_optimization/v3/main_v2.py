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

# #Define symbolic variables for initial state s_0 and control parameters U
# c_0_x, c_0_y, c_0_z = sp.symbols('c_0_x c_0_y c_0_z')
# c_dot_0_x, c_dot_0_y, c_dot_0_z = sp.symbols('c_dot_0_x c_dot_0_y c_dot_0_z')
# alpha, alpha_dot = sp.symbols('alpha_0 alpha_dot_0')
# omega_R, omega_L = sp.symbols('omega_R omega_L')

# T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1 = sp.symbols('T_1 p_0_1_x p_0_1_y p_T_1_x p_T_1_y r_0_1 r_T_1 alpha_ddot_1')
# T_2, p_T_2_x, p_T_2_y, r_T_2, alpha_ddot_2 = sp.symbols('T_2 p_T_2_x p_T_2_y r_T_2 alpha_ddot_2')
# T_3 = sp.symbols('T_3')
# T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, alpha_ddot_4 = sp.symbols('T_4 p_0_4_x p_0_4_y p_T_4_x p_T_4_y r_0_4 r_T_4 alpha_ddot_4')
# T_5, p_T_5_x, p_T_5_y, r_T_5, alpha_ddot_5 = sp.symbols('T_5 p_T_5_x p_T_5_y r_T_5 alpha_ddot_5')
# T_6 = sp.symbols('T_6')
# u_swing_x, u_swing_y = sp.symbols('u_swing_x u_swing_y')
# symbols = (
#     c_0_x, c_0_y, c_0_z,
#     c_dot_0_x, c_dot_0_y, c_dot_0_z,
#     alpha, alpha_dot,
#     omega_R, omega_L,
#     T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1,
#     T_2, p_T_2_x, p_T_2_y, r_T_2, alpha_ddot_2,
#     T_3,
#     T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, alpha_ddot_4,
#     T_5, p_T_5_x, p_T_5_y, r_T_5, alpha_ddot_5,
#     T_6,
#     u_swing_x, u_swing_y, 
#     t
# )

# # Assume your symbolic expressions are defined globally or passed as parameters
# # For demonstration, let's assume a symbolic expression 'poly' for one segment
# c_0_x, c_0_y, c_0_z, c_dot_0_x, c_dot_0_y, c_dot_0_z, alpha, alpha_dot, T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1, ... = symbols('c_0_x c_0_y c_0_z c_dot_0_x c_dot_0_y c_dot_0_z alpha alpha_dot T_1 p_0_1_x p_0_1_y p_T_1_x p_T_1_y r_0_1 r_T_1 alpha_ddot_1 ...')

# # Example of converting one polynomial segment to a numerical function
# poly_func = lambdify([c_0_x, c_0_y, c_0_z, c_dot_0_x, c_dot_0_y, c_dot_0_z, alpha, alpha_dot, 
#                       T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1, ...], poly, 'numpy')

# # Now in your evaluation function, use the numerical functions instead of symbolic substitution
# def evaluate_piecewise_polynomial_segment_numeric(poly_func, initial_state, control_params):
#     # Unpack initial state and control parameters
#     c_0_x_numeric, c_0_y_numeric, c_0_z_numeric, ... = initial_state.c + initial_state.c_dot + (initial_state.alpha, initial_state.alpha_dot)
#     control_values = [control_params[0].duration, control_params[0].p_0[0], control_params[0].p_0[1], control_params[0].p_T[0], control_params[0].p_T[1], control_params[0].r_0, control_params[0].r_T, control_params[0].alpha_ddot, ...]

#     # Evaluate the numerical function
#     result = poly_func(*control_values)
#     return result


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
    #Generate numerical preview
    # preview_schedule = evaluate_preview_schedule_control(lambdified_preview_schedule, s_0, preview_control, t)

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
# def generate_symbolic_multiphase_preview():
#     U = [T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, alpha_ddot_1,
#             T_2, p_T_2_x, p_T_2_y, r_T_2, alpha_ddot_2,
#             T_3,
#             T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, alpha_ddot_4,
#             T_5, p_T_5_x, p_T_5_y, r_T_5, alpha_ddot_5,
#             T_6,
#             u_swing_x, u_swing_y]
    
#     # Convert U to PreviewControl object
#     preview_control = convert_U_to_preview_control(U)

#     # Generate the preview
#     preview_schedule = preview_locomotion.generate_multi_phase_preview(s_0, preview_control)

#     return preview_schedule

# def evaluate_preview_schedule_control(symbolic_preview_schedule: PiecewisePolynomial, initial_state: State, control: PreviewControl):
#     # Generate the preview
#     x_seg = symbolic_preview_schedule.x_segments
#     y_seg = symbolic_preview_schedule.y_segments
#     z_seg = symbolic_preview_schedule.z_segments
#     alpha_seg = symbolic_preview_schedule.alpha_segments
#     xdot_seg = symbolic_preview_schedule.xdot_segments
#     ydot_seg = symbolic_preview_schedule.ydot_segments
#     zdot_seg = symbolic_preview_schedule.zdot_segments
#     alpha_dot_seg = symbolic_preview_schedule.alpha_dot_segments

#     x_segments_eval = evaluate_piecewise_polynomial_segment(x_seg, initial_state, control)
#     y_segments_eval = evaluate_piecewise_polynomial_segment(y_seg, initial_state, control)
#     z_segments_eval = evaluate_piecewise_polynomial_segment(z_seg, initial_state, control)
#     alpha_segments_eval = evaluate_piecewise_polynomial_segment(alpha_seg, initial_state, control)
#     xdot_segments_eval = evaluate_piecewise_polynomial_segment(xdot_seg, initial_state, control)
#     ydot_segments_eval = evaluate_piecewise_polynomial_segment(ydot_seg, initial_state, control)
#     zdot_segments_eval = evaluate_piecewise_polynomial_segment(zdot_seg, initial_state, control)
#     alpha_dot_segments_eval = evaluate_piecewise_polynomial_segment(alpha_dot_seg, initial_state, control)

#     preview_schedule_eval = PiecewisePolynomial(x_segments_eval, y_segments_eval, z_segments_eval, alpha_segments_eval, 
#                                                 xdot_segments_eval, ydot_segments_eval, zdot_segments_eval, alpha_dot_segments_eval)

#     return preview_schedule_eval

# def evaluate_preview_schedule_control(preview_schedule_lambdified , initial_state: State, control: PreviewControl, t_value):
#     numerical_values = (
#         initial_state.c[0], initial_state.c[1], initial_state.c[2],
#         initial_state.c_dot[0], initial_state.c_dot[1], initial_state.c_dot[2],
#         initial_state.alpha, initial_state.alpha_dot,
#         0, 0,
#         control.params[0].duration, control.params[0].p_0[0], control.params[0].p_0[1], control.params[0].p_T[0], control.params[0].p_T[1], 
#         control.params[0].r_0, control.params[0].r_T, control.params[0].alpha_ddot,
#         control.params[1].duration, control.params[1].p_T[0], control.params[1].p_T[1], control.params[1].r_T, control.params[1].alpha_ddot,
#         control.params[2].duration,
#         control.params[3].duration, control.params[3].p_0[0], control.params[3].p_0[1], control.params[3].p_T[0], control.params[3].p_T[1],
#         control.params[3].r_0, control.params[3].r_T, control.params[3].alpha_ddot,
#         control.params[4].duration, control.params[4].p_T[0], control.params[4].p_T[1], control.params[4].r_T, control.params[4].alpha_ddot,
#         control.params[5].duration,
#         0, 0
#     )
    
#     evaluated_preview_schedule = PiecewisePolynomial()

#     # Adjust the loop to call lambdified functions with 't_value'
#     for segment_type, lambdified_segments in preview_schedule_lambdified.items():
#         evaluated_segments = []
#         for lambdified_segment in lambdified_segments:
#             # Append 't_value' to 'numerical_values' for each function call
#             evaluated_polynomial = lambdified_segment['polynomial'](*(numerical_values + (t_value,)))
#             evaluated_start_time = lambdified_segment['start_time'](*(numerical_values + (t_value,)))
#             evaluated_end_time = lambdified_segment['end_time'](*(numerical_values + (t_value,)))
#             evaluated_initial_value = lambdified_segment['initial_value'](*(numerical_values + (t_value,)))
#             evaluated_final_value = lambdified_segment['final_value'](*(numerical_values + (t_value,)))
            
#             # Construct and append evaluated segment
#             evaluated_segment = {
#                 'polynomial': evaluated_polynomial,
#                 'start_time': evaluated_start_time,
#                 'end_time': evaluated_end_time,
#                 'initial_value': evaluated_initial_value,
#                 'final_value': evaluated_final_value
#             }
#             evaluated_segments.append(evaluated_segment)
        
#         setattr(evaluated_preview_schedule, segment_type, evaluated_segments)

#     return evaluated_preview_schedule

# def lambdify_all_segments(piecewisePolynomial, symbols):
#     lambdified_segments = {
#         'x': [],
#         'y': [],
#         'z': [],
#         'alpha': [],
#         'xdot': [],
#         'ydot': [],
#         'zdot': [],
#         'alpha_dot': []
#     }

#     for attribute, segments in lambdified_segments.items():
#         for segment in getattr(piecewisePolynomial, f"{attribute}_segments"):
#             lambdified = lambdify_segment(segment, symbols)
#             segments.append(lambdified)

#     return lambdified_segments


# def lambdify_segment(segment, symbols):
#     """
#     Converts the symbolic expressions of a PiecewisePolynomialSegment to numerical functions.

#     :param segment: A PiecewisePolynomialSegment object.
#     :param symbols: The symbolic variables in the expressions.
#     :return: A dictionary of lambdified functions.
#     """
#     return {
#         'polynomial': lambdify(symbols, segment.polynomial, modules=['numpy']),
#         'start_time': lambdify(symbols, segment.start_time, modules=['numpy']),
#         'end_time': lambdify(symbols, segment.end_time, modules=['numpy']),
#         'initial_value': lambdify(symbols, segment.initial_value, modules=['numpy']),
#         'final_value': lambdify(symbols, segment.final_value, modules=['numpy'])
#     }


# def evaluate_piecewise_polynomial_segment(segments, initial_state: State, control: PreviewControl):
#     c_0_x_numeric, c_0_y_numeric, c_0_z_numeric = initial_state.c
#     c_dot_0_x_numeric, c_dot_0_y_numeric, c_dot_0_z_numeric = initial_state.c_dot
#     alpha_numeric, alpha_dot_numeric= initial_state.alpha, initial_state.alpha_dot

#     T_1_numeric = control.params[0].duration
#     p_0_1_x_numeric, p_0_1_y_numeric = control.params[0].p_0
#     p_T_1_x_numeric, p_T_1_y_numeric = control.params[0].p_T
#     r_0_1_numeric, r_T_1_numeric, alpha_ddot_1_numeric = control.params[0].r_0, control.params[0].r_T, control.params[0].alpha_ddot
    
#     T_2_numeric = control.params[1].duration
#     p_T_2_x_numeric, p_T_2_y_numeric = control.params[1].p_T
#     r_T_2_numeric = control.params[1].r_T
#     alpha_ddot_2_numeric = control.params[1].alpha_ddot

#     T_3_numeric = control.params[2].duration
    
#     T_4_numeric = control.params[3].duration
#     p_0_4_x_numeric, p_0_4_y_numeric = control.params[3].p_0
#     p_T_4_x_numeric, p_T_4_y_numeric = control.params[3].p_T
#     r_0_4_numeric, r_T_4_numeric, alpha_ddot_4_numeric = control.params[3].r_0, control.params[3].r_T, control.params[3].alpha_ddot
    
#     T_5_numeric = control.params[4].duration
#     p_T_5_x_numeric, p_T_5_y_numeric = control.params[4].p_T
#     r_T_5_numeric = control.params[4].r_T
#     alpha_ddot_5_numeric = control.params[4].alpha_ddot
    
#     T_6_numeric = control.params[5].duration
#     # u_swing_x_numeric, u_swing_y_numeric = control.params[5].u_swing_x, control.params[5].u_swing_y
#     u_swing_x_numeric, u_swing_y_numeric = 0, 0

#     new_segments = []

#     for segment in segments:
#         poly = segment.polynomial
#         start_time_segment = segment.start_time
#         end_time_segment = segment.end_time
#         initial_value = segment.initial_value
#         final_value = segment.final_value

#         poly = poly.subs({c_0_x: c_0_x_numeric, c_0_y: c_0_y_numeric, c_0_z: c_0_z_numeric, c_dot_0_x: c_dot_0_x_numeric, c_dot_0_y: c_dot_0_y_numeric, c_dot_0_z: c_dot_0_z_numeric, alpha: alpha_numeric, alpha_dot: alpha_dot_numeric, omega_R: 0, omega_L: 0,
#                             T_1: T_1_numeric, p_0_1_x: p_0_1_x_numeric, p_0_1_y: p_0_1_y_numeric, p_T_1_x: p_T_1_x_numeric, p_T_1_y: p_T_1_y_numeric, r_0_1: r_0_1_numeric, r_T_1: r_T_1_numeric, alpha_ddot_1: alpha_ddot_1_numeric,
#                             T_2: T_2_numeric, p_T_2_x: p_T_2_x_numeric, p_T_2_y: p_T_2_y_numeric, r_T_2: r_T_2_numeric, alpha_ddot_2: alpha_ddot_2_numeric,
#                             T_3: T_3_numeric,
#                             T_4: T_4_numeric, p_0_4_x: p_0_4_x_numeric, p_0_4_y: p_0_4_y_numeric, p_T_4_x: p_T_4_x_numeric, p_T_4_y: p_T_4_y_numeric, r_0_4: r_0_4_numeric, r_T_4: r_T_4_numeric, alpha_ddot_4: alpha_ddot_4_numeric,
#                             T_5: T_5_numeric, p_T_5_x: p_T_5_x_numeric, p_T_5_y: p_T_5_y_numeric, r_T_5: r_T_5_numeric, alpha_ddot_5: alpha_ddot_5_numeric,
#                             T_6: T_6_numeric,
#                             u_swing_x: u_swing_x_numeric, u_swing_y: u_swing_y_numeric}) if poly != 0 else 0
        
#         start_time_segment = start_time_segment.subs({c_0_x: c_0_x_numeric, c_0_y: c_0_y_numeric, c_0_z: c_0_z_numeric, c_dot_0_x: c_dot_0_x_numeric, c_dot_0_y: c_dot_0_y_numeric, c_dot_0_z: c_dot_0_z_numeric, alpha: alpha_numeric, alpha_dot: alpha_dot_numeric, omega_R: 0, omega_L: 0,
#                             T_1: T_1_numeric, p_0_1_x: p_0_1_x_numeric, p_0_1_y: p_0_1_y_numeric, p_T_1_x: p_T_1_x_numeric, p_T_1_y: p_T_1_y_numeric, r_0_1: r_0_1_numeric, r_T_1: r_T_1_numeric, alpha_ddot_1: alpha_ddot_1_numeric,
#                             T_2: T_2_numeric, p_T_2_x: p_T_2_x_numeric, p_T_2_y: p_T_2_y_numeric, r_T_2: r_T_2_numeric, alpha_ddot_2: alpha_ddot_2_numeric,
#                             T_3: T_3_numeric,
#                             T_4: T_4_numeric, p_0_4_x: p_0_4_x_numeric, p_0_4_y: p_0_4_y_numeric, p_T_4_x: p_T_4_x_numeric, p_T_4_y: p_T_4_y_numeric, r_0_4: r_0_4_numeric, r_T_4: r_T_4_numeric, alpha_ddot_4: alpha_ddot_4_numeric,
#                             T_5: T_5_numeric, p_T_5_x: p_T_5_x_numeric, p_T_5_y: p_T_5_y_numeric, r_T_5: r_T_5_numeric, alpha_ddot_5: alpha_ddot_5_numeric,
#                             T_6: T_6_numeric,
#                             u_swing_x: u_swing_x_numeric, u_swing_y: u_swing_y_numeric}) if start_time_segment != 0 else 0
        
#         end_time_segment = end_time_segment.subs({c_0_x: c_0_x_numeric, c_0_y: c_0_y_numeric, c_0_z: c_0_z_numeric, c_dot_0_x: c_dot_0_x_numeric, c_dot_0_y: c_dot_0_y_numeric, c_dot_0_z: c_dot_0_z_numeric, alpha: alpha_numeric, alpha_dot: alpha_dot_numeric, omega_R: 0, omega_L: 0,
#                             T_1: T_1_numeric, p_0_1_x: p_0_1_x_numeric, p_0_1_y: p_0_1_y_numeric, p_T_1_x: p_T_1_x_numeric, p_T_1_y: p_T_1_y_numeric, r_0_1: r_0_1_numeric, r_T_1: r_T_1_numeric, alpha_ddot_1: alpha_ddot_1_numeric,
#                             T_2: T_2_numeric, p_T_2_x: p_T_2_x_numeric, p_T_2_y: p_T_2_y_numeric, r_T_2: r_T_2_numeric, alpha_ddot_2: alpha_ddot_2_numeric,
#                             T_3: T_3_numeric,
#                             T_4: T_4_numeric, p_0_4_x: p_0_4_x_numeric, p_0_4_y: p_0_4_y_numeric, p_T_4_x: p_T_4_x_numeric, p_T_4_y: p_T_4_y_numeric, r_0_4: r_0_4_numeric, r_T_4: r_T_4_numeric, alpha_ddot_4: alpha_ddot_4_numeric,
#                             T_5: T_5_numeric, p_T_5_x: p_T_5_x_numeric, p_T_5_y: p_T_5_y_numeric, r_T_5: r_T_5_numeric, alpha_ddot_5: alpha_ddot_5_numeric,
#                             T_6: T_6_numeric,
#                             u_swing_x: u_swing_x_numeric, u_swing_y: u_swing_y_numeric}) if end_time_segment != 0 else 0
        
#         initial_value = initial_value.subs({c_0_x: c_0_x_numeric, c_0_y: c_0_y_numeric, c_0_z: c_0_z_numeric, c_dot_0_x: c_dot_0_x_numeric, c_dot_0_y: c_dot_0_y_numeric, c_dot_0_z: c_dot_0_z_numeric, alpha: alpha_numeric, alpha_dot: alpha_dot_numeric, omega_R: 0, omega_L: 0,
#                             T_1: T_1_numeric, p_0_1_x: p_0_1_x_numeric, p_0_1_y: p_0_1_y_numeric, p_T_1_x: p_T_1_x_numeric, p_T_1_y: p_T_1_y_numeric, r_0_1: r_0_1_numeric, r_T_1: r_T_1_numeric, alpha_ddot_1: alpha_ddot_1_numeric,
#                             T_2: T_2_numeric, p_T_2_x: p_T_2_x_numeric, p_T_2_y: p_T_2_y_numeric, r_T_2: r_T_2_numeric, alpha_ddot_2: alpha_ddot_2_numeric,
#                             T_3: T_3_numeric,
#                             T_4: T_4_numeric, p_0_4_x: p_0_4_x_numeric, p_0_4_y: p_0_4_y_numeric, p_T_4_x: p_T_4_x_numeric, p_T_4_y: p_T_4_y_numeric, r_0_4: r_0_4_numeric, r_T_4: r_T_4_numeric, alpha_ddot_4: alpha_ddot_4_numeric,
#                             T_5: T_5_numeric, p_T_5_x: p_T_5_x_numeric, p_T_5_y: p_T_5_y_numeric, r_T_5: r_T_5_numeric, alpha_ddot_5: alpha_ddot_5_numeric,
#                             T_6: T_6_numeric,
#                             u_swing_x: u_swing_x_numeric, u_swing_y: u_swing_y_numeric}) if initial_value != 0 else 0
        
#         final_value = final_value.subs({c_0_x: c_0_x_numeric, c_0_y: c_0_y_numeric, c_0_z: c_0_z_numeric, c_dot_0_x: c_dot_0_x_numeric, c_dot_0_y: c_dot_0_y_numeric, c_dot_0_z: c_dot_0_z_numeric, alpha: alpha_numeric, alpha_dot: alpha_dot_numeric, omega_R: 0, omega_L: 0,
#                             T_1: T_1_numeric, p_0_1_x: p_0_1_x_numeric, p_0_1_y: p_0_1_y_numeric, p_T_1_x: p_T_1_x_numeric, p_T_1_y: p_T_1_y_numeric, r_0_1: r_0_1_numeric, r_T_1: r_T_1_numeric, alpha_ddot_1: alpha_ddot_1_numeric,
#                             T_2: T_2_numeric, p_T_2_x: p_T_2_x_numeric, p_T_2_y: p_T_2_y_numeric, r_T_2: r_T_2_numeric, alpha_ddot_2: alpha_ddot_2_numeric,
#                             T_3: T_3_numeric,
#                             T_4: T_4_numeric, p_0_4_x: p_0_4_x_numeric, p_0_4_y: p_0_4_y_numeric, p_T_4_x: p_T_4_x_numeric, p_T_4_y: p_T_4_y_numeric, r_0_4: r_0_4_numeric, r_T_4: r_T_4_numeric, alpha_ddot_4: alpha_ddot_4_numeric,
#                             T_5: T_5_numeric, p_T_5_x: p_T_5_x_numeric, p_T_5_y: p_T_5_y_numeric, r_T_5: r_T_5_numeric, alpha_ddot_5: alpha_ddot_5_numeric,
#                             T_6: T_6_numeric,
#                             u_swing_x: u_swing_x_numeric, u_swing_y: u_swing_y_numeric}) if final_value != 0 else 0
        
#         new_segments.append(PiecewisePolynomialSegment(poly, start_time_segment, end_time_segment, initial_value, final_value))

#     return new_segments
 

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
    # preview_schedule_sym = generate_symbolic_multiphase_preview()
    
    # lambdified_preview_schedule = lambdify_all_segments(preview_schedule_sym, symbols)
   

    # # Define control input, u

    # Initial guess for U
    # U = [u_1 u_2 u_3 u_4 u_5 u_6 u_swing]
    #
    #   = [T_1  ,  p_{0,1}     ,   p_{T,1/0,2} ,  r_{0,1}     ,   r_{T,1/0,2} ,  alpha_ddot_1
    #      T_2  ,  p_{T,1/0,2} ,   p_{T,2}     ,  r_{T,1/0,2} ,   r_{T,2}     ,  alpha_ddot_2
    #      T_3  ,
    #      T_4  ,  p_{0,4}     ,   p_{T,4/0,5} ,  r_{0,4}     ,   r_{T,4/0,5} ,  alpha_ddot_4
    #      T_5  ,  p_{T,4/0,5} ,   p_{T,5}     ,  r_{T,4/0,5} ,   r_{T,5}     ,  alpha_ddot_5
    #      T_6  ,
    #      u_swing]
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