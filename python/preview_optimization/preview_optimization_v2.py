from sympy import symbols, Matrix, exp, cos, sin, diff

def compute_p(t, T, p_0, p_T):
    p_x = (t/T)*(p_T[0] - p_0[0]) + p_0[0]
    p_y = (t/T)*(p_T[1] - p_0[1]) + p_0[1]
    return Matrix([p_x, p_y])

def compute_r(t, T, r_0, r_T):
    r = (t/T)*(r_T - r_0) + r_0
    return r

def stance_dynamics(t, T, alpha_freq, alpha_0, alpha_dot_0, alpha_ddot, d1, d2, x_0, y_0, z_0, p_0_x, p_0_y, p_T_x, p_T_y, r_0, r_T, x_dot_0, y_dot_0, Omega_R=0, Omega_L=0):
    p = compute_p(t, T, [p_0_x, p_0_y], [p_T_x, p_T_y])
    r = compute_r(t, T, r_0, r_T)

    beta_x_1 = ((x_0 - p_0_x)/2) + ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha_freq*T))
    beta_x_2 = ((x_0 - p_0_x)/2) - ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha_freq*T))
    beta_y_1 = ((y_0 - p_0_y)/2) + ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha_freq*T))
    beta_y_2 = ((y_0 - p_0_y)/2) - ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha_freq*T))

    x_stance = beta_x_1*exp(alpha_freq*t) + beta_x_2*exp(-alpha_freq*t) + p[0]
    y_stance = beta_y_1*exp(alpha_freq*t) + beta_y_2*exp(-alpha_freq*t) + p[1]
    z_stance = d1*cos(omega*t) + d2*sin(omega*t) + r + (g/omega**2)

    c_stance = Matrix([x_stance, y_stance, z_stance])

    x_dot_stance = diff(x_stance, t)
    y_dot_stance = diff(y_stance, t)
    z_dot_stance = diff(z_stance, t)

    c_dot_stance = Matrix([x_dot_stance, y_dot_stance, z_dot_stance])

    alpha_stance = alpha_0 + alpha_dot_0*t + (1/2)*alpha_ddot[2]*t**2

    alpha_dot_stance = alpha_dot_0 + alpha_ddot[2]*t    # hard-coded, faster than diff(alpha_stance, t)

    # Append everything into a single vector
    state_vector = Matrix([c_stance[0], c_stance[1], c_stance[2], c_dot_stance[0], c_dot_stance[1], c_dot_stance[2], alpha_stance, alpha_dot_stance, Omega_R, Omega_L])

    return state_vector

# def stance_dynamics(t, T, alpha, alpha_0, alpha_dot_0, alpha_ddot, d1, d2, x_0, y_0, z_0, p_0_x, p_0_y, p_T_x, p_T_y, r_0, r_T, x_dot_0, y_dot_0):
#     p_x = (t/T)*(p_T_x - p_0_x) + p_0_x
#     p_y = (t/T)*(p_T_y - p_0_y) + p_0_y
#     r = (t/T)*(r_T - r_0) + r_0

#     beta_x_1 = ((x_0 - p_0_x)/2) + ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha*T))
#     beta_x_2 = ((x_0 - p_0_x)/2) - ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha*T))
#     beta_y_1 = ((y_0 - p_0_y)/2) + ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha*T))
#     beta_y_2 = ((y_0 - p_0_y)/2) - ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha*T))

#     x_stance = beta_x_1*exp(alpha*t) + beta_x_2*exp(-alpha*t) + p_x
#     y_stance = beta_y_1*exp(alpha*t) + beta_y_2*exp(-alpha*t) + p_y
#     z_stance = d1*cos(omega*t) + d2*sin(omega*t) + r + (g/omega**2)

#     c_stance = Matrix([x_stance, y_stance, z_stance])

#     return Matrix([x_stance, y_stance, z_stance])

def flight_dynamics(t, g, s_0):
    c_double_dot_gravity = [0, 0, g]
    c_flight = Matrix([s_0[5] + s_0[8]*t + (1/2)*c_double_dot_gravity[0]*t**2,
                       s_0[6] + s_0[9]*t + (1/2)*c_double_dot_gravity[1]*t**2,
                       s_0[7] + s_0[10]*t + (1/2)*c_double_dot_gravity[2]*t**2])
    
    alpha_flight = s_0[11] + s_0[12]*t

    return c_flight, alpha_flight

# Define symbolic variables
t, T, g, h, k, m, omega, alpha, alpha_0, alpha_dot_0, alpha_ddot, d1, d2 = symbols('t T g h k m omega alpha alpha_0 alpha_dot_0 alpha_ddot d1 d2')

# Define control parameters for each phase
U = symbols('T1 p_0_1 p_T_1 r_0_1 r_T_1 alpha_ddot_1 T2 p_0_2 p_T_2 r_0_2 r_T_2 alpha_ddot_2 T3 p_0_3 p_T_3 r_0_3 r_T_3 alpha_ddot_3 T4 p_0_4 p_T_4 r_0_4 r_T_4 alpha_ddot_4 T5 p_0_5 p_T_5 r_0_5 r_T_5 alpha_ddot_5 T6 p_0_6 p_T_6 r_0_6 r_T_6 alpha_ddot_6', real=True)

# Extract individual control parameters for each phase
params_per_phase = [U[i:i+6] for i in range(0, len(U), 6)]

# Define symbolic variables for initial conditions
s_0 = Matrix(symbols('x_0 y_0 z_0 x_dot_0 y_dot_0 alpha_0 alpha_dot_0', real=True))

"""
Structure of s_0:
- s_0[0]: x-coordinate
- s_0[1]: y-coordinate
- s_0[2]: z-coordinate
- s_0[3]: x-velocity
- s_0[4]: y-velocity
- s_0[5]: alpha_0 (initial alpha)
- s_0[6]: alpha_dot_0 (initial alpha dot)
"""

# Initialize the trajectory sequence
S_t = []

# Perform symbolic simulation for each phase
for i in range(6):
    # Extract control parameters for the current phase
    params = params_per_phase[i]
    T_i, p_0_i, p_T_i, r_0_i, r_T_i, alpha_ddot_i = params

    # Concatenate control parameters for continuity
    if i > 0:
        p_0_i = params_per_phase[i-1][2]

    # Check the type of phase and compute dynamics accordingly
    if i % 3 == 0:  # Double stance phase
        stance_result = stance_dynamics(t, T_i, alpha, alpha_0, alpha_dot_0, alpha_ddot_i, d1, d2, s_0[0], s_0[1], s_0[2], p_0_i[0], p_0_i[1], p_T_i[0], p_T_i[1], r_0_i, r_T_i, s_0[3], s_0[4])
        S_t.extend(stance_result)
    elif i % 3 == 1:  # Single stance phase
        stance_result = stance_dynamics(t, T_i, alpha, alpha_0, alpha_dot_0, alpha_ddot_i, d1, d2, s_0[0], s_0[1], s_0[2], p_0_i[0], p_0_i[1], p_T_i[0], p_T_i[1], r_0_i, r_T_i, s_0[3], s_0[4])
        S_t.extend(stance_result)
    else:  # Flight phase
        flight_result, alpha_flight_result = flight_dynamics(t, g, s_0)
        S_t.extend(flight_result)
        S_t.append(alpha_flight_result)

# Display symbolic expressions for the entire preview schedule
print("Preview Schedule:")
print("S(t) =", S_t)
