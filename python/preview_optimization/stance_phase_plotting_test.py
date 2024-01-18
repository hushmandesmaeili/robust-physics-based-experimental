import numpy as np
from sympy import Interval, Piecewise, symbols
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import lambdify

from plant import Plant
from state_vector import StateVector
from control_input import ControlInput


# Assuming t_symbolic is your symbolic time variable
t_symbolic = symbols('t')

# Define initial state (s_0), control parameters (u), and initial height (h_initial)
s_0 = StateVector(x=0, y=0, z=1.5, x_dot=2, y_dot=0, z_dot=0, alpha=0, alpha_dot=0, Omega_R=0, Omega_L=0)
u_1 = ControlInput(0.15, [0.1, 0], [0.25, 0], 1.6, 1.75, [0, 0, 0])  # Example values, adjust as needed
h_initial = s_0.z # Example value, adjust as needed

plant = Plant()

# Get the symbolic trajectory for the stance phase
symbolic_trajectory = plant.stance_dynamics(t_symbolic, s_0, u_1, h_initial)

# Extract symbolic expressions for COM position (c(t))
c_t_x = symbolic_trajectory.x
c_t_y = symbolic_trajectory.y
c_t_z = symbolic_trajectory.z

# Extracting p0 and pT values from control input
p0_x, p0_y = u_1.p_0
pT_x, pT_y = u_1.p_T
r0, rT = u_1.r_0, u_1.r_T

condition_trajectory = (t_symbolic >= 0) & (t_symbolic <= u_1.T)

# Define the piecewise functions for COM position with bounds
c_t_x_bound = Piecewise((c_t_x, condition_trajectory), (0, True))
c_t_y_bound = Piecewise((c_t_y, condition_trajectory), (0, True))
c_t_z_bound = Piecewise((c_t_z, condition_trajectory), (0, True))

# Convert piecewise symbolic expressions to numeric functions
c_t_x_bound_numeric = lambdify(t_symbolic, c_t_x_bound, 'numpy')
c_t_y_bound_numeric = lambdify(t_symbolic, c_t_y_bound, 'numpy')
c_t_z_bound_numeric = lambdify(t_symbolic, c_t_z_bound, 'numpy')

# Evaluate numeric functions
t_values_bound = np.linspace(0, float(u_1.T), 100)  # Adjust the number of points as needed
c_t_x_values_bound = c_t_x_bound_numeric(t_values_bound) if not c_t_x_bound.equals(0) else [0] * 100
c_t_y_values_bound = c_t_y_bound_numeric(t_values_bound) if not c_t_y_bound.equals(0) else [0] * 100
c_t_z_values_bound = c_t_z_bound_numeric(t_values_bound) if not c_t_z_bound.equals(0) else [0] * 100


# Plot the numeric expressions in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(c_t_x_values_bound, c_t_y_values_bound, c_t_z_values_bound, label='COM Position (c(t))', color='blue')
ax.scatter([float(p0_x)], [float(p0_y)], [float(0)], color='red', label='COP Initial Position (p0)')
ax.scatter([float(pT_x)], [float(pT_y)], [float(0)], color='green', label='COP Final Position (pT)')

# # Plot lines for initial and final rest lengths
# initial_rest_length = np.array([float(p0_x), float(p0_y), 0]) + r0 * np.array([float(s_0.x - p0_x), float(s_0.y - p0_y), 0]) / sqrt((s_0.x - p0_x)**2 + (s_0.y - p0_y)**2)
# final_rest_length = np.array([float(pT_x), float(pT_y), 0]) + rT * np.array([float(final_state.x - pT_x), float(final_state.y - pT_y), 0]) / sqrt((final_state.x - pT_x)**2 + (final_state.y - pT_y)**2)

# ax_bound.plot([float(p0_x), initial_rest_length[0]], [float(p0_y), initial_rest_length[1]], [0, initial_rest_length[2]], label='Initial Rest Length (r0)', linestyle='dashed', color='orange')
# ax_bound.plot([float(pT_x), final_rest_length[0]], [float(pT_y), final_rest_length[1]], [0, final_rest_length[2]], label='Final Rest Length (rT)', linestyle='dashed', color='purple')

# Set bounds for the plot
ax.set_xlim([0, 1])  # Adjust the x-axis bounds as needed
ax.set_ylim([0, 1])  # Adjust the y-axis bounds as needed
ax.set_zlim([0, 3])  # Adjust the z-axis bounds as needed

# Customize the 3D plot
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('Stance Phase Symbolic Trajectory (3D)')
ax.legend()
plt.show()

