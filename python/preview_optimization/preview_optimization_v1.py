from sympy import symbols, Eq, solve, sqrt, cos, sin, exp, Function, diff, symbols, Matrix

# Define symbolic variables
t, T, g, h, k, m, omega, alpha, alpha_0, alpha_dot_0, alpha_ddot, d1, d2 = symbols('t T g h k m omega alpha alpha_0 alpha_dot_0 alpha_ddot d1 d2')

# Stance Dynamics
x_0, y_0, z_0, p_0_x, p_0_y, p_T_x, p_T_y, r_0, r_T, x, y, z, x_dot_0, y_dot_0 = symbols('x_0 y_0 z_0 p_0_x p_0_y p_T_x p_T_y r_0 r_T x y z x_dot_0 y_dot_0')
p_x = (t/T)*(p_T_x - p_0_x) + p_0_x
p_y = (t/T)*(p_T_y - p_0_y) + p_0_y
r = (t/T)*(r_T - r_0) + r_0

beta_x_1 = ((x_0 - p_0_x)/2) + ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha*T))
beta_x_2 = ((x_0 - p_0_x)/2) - ((x_dot_0*T - (p_T_x - p_0_x))/(2*alpha*T))
beta_y_1 = ((y_0 - p_0_y)/2) + ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha*T))
beta_y_2 = ((y_0 - p_0_y)/2) - ((y_dot_0*T - (p_T_y - p_0_y))/(2*alpha*T))

x_stance = beta_x_1*exp(alpha*t) + beta_x_2*exp(-alpha*t) + p_x
y_stance = beta_y_1*exp(alpha*t) + beta_y_2*exp(-alpha*t) + p_y
z_stance = d1*cos(omega*t) + d2*sin(omega*t) + r + (g/omega**2)
c_stance = Matrix([x_stance,
                   y_stance,
                   z_stance])

alpha_stance = alpha_0 + alpha_dot_0*t + (1/2)*alpha_ddot*t**2

# Flight Dynamics
c_0_x, c_0_y, c_0_z, c_dot_0_x, c_dot_0_y, c_dot_0_z, c_x, c_y, c_z = symbols('c_0_x c_0_y c_0_z c_dot_0_x c_dot_0_y c_dot_0_z c_x c_y c_z')
c_double_dot_gravity = [0, 0, g]
c_flight = Matrix([c_0_x + c_dot_0_x*t + (1/2)*c_double_dot_gravity[0]*t**2,
                   c_0_y + c_dot_0_y*t + (1/2)*c_double_dot_gravity[1]*t**2,
                   c_0_z + c_dot_0_z*t + (1/2)*c_double_dot_gravity[2]*t**2])
alpha_flight = alpha_0 + alpha_dot_0*t

# # Display symbolic expressions
# print("Stance Dynamics:")
# print("x(t) =", x_stance)
# print("y(t) =", y_stance)
# print("z(t) =", z_stance)

# print("\nFlight Dynamics:")
# print("c(t)_flight =", c_flight)
# print("alpha(t) =", alpha_0 + alpha_dot_0*t)

# Define symbolic variables
t, T, g, h, k, m, omega, alpha, alpha_0, alpha_dot_0, alpha_ddot, d1, d2 = symbols('t T g h k m omega alpha alpha_0 alpha_dot_0 alpha_ddot d1 d2')

# Define control parameters for each phase
U = symbols('T1 p_0_1 p_T_1 r_0_1 r_T_1 alpha_ddot_1 T2 p_0_2 p_T_2 r_0_2 r_T_2 alpha_ddot_2 T3 p_0_3 p_T_3 r_0_3 r_T_3 alpha_ddot_3 T4 p_0_4 p_T_4 r_0_4 r_T_4 alpha_ddot_4 T5 p_0_5 p_T_5 r_0_5 r_T_5 alpha_ddot_5 T6 p_0_6 p_T_6 r_0_6 r_T_6 alpha_ddot_6', real=True)

# Extract individual control parameters for each phase
params_per_phase = [U[i:i+6] for i in range(0, len(U), 6)]

# Define symbolic variables for initial conditions
s_0 = Matrix(symbols('x_0 y_0 z_0 x_dot_0 y_dot_0 alpha_0 alpha_dot_0', real=True))

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
        p_x_i = (t/T_i)*(p_T_i[0] - p_0_i[0]) + p_0_i[0]
        p_y_i = (t/T_i)*(p_T_i[1] - p_0_i[1]) + p_0_i[1]
        r_i = (t/T_i)*(r_T_i - r_0_i) + r_0_i

        beta_x_1_i = ((s_0[0] - p_0_i[0])/2) + ((s_0[3]*T_i - (p_T_i[0] - p_0_i[0]))/(2*alpha*T_i))
        beta_x_2_i = ((s_0[0] - p_0_i[0])/2) - ((s_0[3]*T_i - (p_T_i[0] - p_0_i[0]))/(2*alpha*T_i))
        beta_y_1_i = ((s_0[1] - p_0_i[1])/2) + ((s_0[4]*T_i - (p_T_i[1] - p_0_i[1]))/(2*alpha*T_i))
        beta_y_2_i = ((s_0[1] - p_0_i[1])/2) - ((s_0[4]*T_i - (p_T_i[1] - p_0_i[1]))/(2*alpha*T_i))

        x_stance_i = beta_x_1_i*exp(alpha*t) + beta_x_2_i*exp(-alpha*t) + p_x_i
        y_stance_i = beta_y_1_i*exp(alpha*t) + beta_y_2_i*exp(-alpha*t) + p_y_i
        z_stance_i = d1*cos(omega*t) + d2*sin(omega*t) + r_i + (g/omega**2)
        alpha_stance_i = alpha_0 + alpha_dot_0*t + (1/2)*alpha_ddot_i*t**2

        S_t.extend([x_stance_i, y_stance_i, z_stance_i, alpha_stance_i])
    elif i % 3 == 1:  # Single stance phase
        # Add single stance dynamics here
        # Same as double-stance
        p_x_i = (t/T_i)*(p_T_i[0] - p_0_i[0]) + p_0_i[0]
        p_y_i = (t/T_i)*(p_T_i[1] - p_0_i[1]) + p_0_i[1]
        r_i = (t/T_i)*(r_T_i - r_0_i) + r_0_i

        beta_x_1_i = ((s_0[0] - p_0_i[0])/2) + ((s_0[3]*T_i - (p_T_i[0] - p_0_i[0]))/(2*alpha*T_i))
        beta_x_2_i = ((s_0[0] - p_0_i[0])/2) - ((s_0[3]*T_i - (p_T_i[0] - p_0_i[0]))/(2*alpha*T_i))
        beta_y_1_i = ((s_0[1] - p_0_i[1])/2) + ((s_0[4]*T_i - (p_T_i[1] - p_0_i[1]))/(2*alpha*T_i))
        beta_y_2_i = ((s_0[1] - p_0_i[1])/2) - ((s_0[4]*T_i - (p_T_i[1] - p_0_i[1]))/(2*alpha*T_i))

        x_stance_i = beta_x_1_i*exp(alpha*t) + beta_x_2_i*exp(-alpha*t) + p_x_i
        y_stance_i = beta_y_1_i*exp(alpha*t) + beta_y_2_i*exp(-alpha*t) + p_y_i
        z_stance_i = d1*cos(omega*t) + d2*sin(omega*t) + r_i + (g/omega**2)
        alpha_stance_i = alpha_0 + alpha_dot_0*t + (1/2)*alpha_ddot_i*t**2

        S_t.extend([x_stance_i, y_stance_i, z_stance_i, alpha_stance_i])
    else:  # Flight phase
        # Add flight dynamics here
        c_x_i = s_0[5] + s_0[8]*t + (1/2)*g*t**2
        c_y_i = s_0[6] + s_0[9]*t + (1/2)*g*t**2
        c_z_i = s_0[7] + s_0[10]*t + (1/2)*g*t**2
        alpha_flight_i = alpha_0 + alpha_dot_0*t

        S_t.extend([c_x_i, c_y_i, c_z_i, alpha_flight_i])

# Display symbolic expressions for the entire preview schedule
print("Preview Schedule:")
print("S(t) =", S_t)

