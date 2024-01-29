import numpy as np
from sympy import Interval, Piecewise, symbols
import matplotlib.pyplot as plt
from sympy import lambdify

from plant import Plant
from state_vector import StateVector
from control_input import ControlInput

# Assuming t_symbolic is your symbolic time variable
t_symbolic = symbols('t')

def compute_numeric_values(symbolic_trajectory, t_values, condition_trajectory):
    # Define the piecewise functions for COM position and velocity with bounds
    c_t_z_bound = Piecewise((symbolic_trajectory.z, condition_trajectory), (0, True))

    # Convert piecewise symbolic expressions to numeric functions
    c_t_z_bound_numeric = lambdify(t_symbolic, c_t_z_bound, 'numpy')

    # Evaluate numeric function for z-values
    c_t_z_values_bound = c_t_z_bound_numeric(t_values) if not c_t_z_bound.equals(0) else np.zeros_like(t_values)

    return c_t_z_values_bound

def plot_z_values(t_values, c_t_z_values, label, color):
    plt.plot(t_values, c_t_z_values, label=f'{label} (c_z(t))', color=color)
    plt.xlabel('Time')
    plt.ylabel('Z Position')
    plt.title(f'{label} Phase - Z Position')
    plt.legend()
    plt.show()

def main():
    # Define initial state (s_0), control parameters (u_1), and initial height (h_initial)
    s_0 = StateVector(x=0.57, y=0, z=1.611, x_dot=1.5, y_dot=0, z_dot=-1.75, alpha=0, alpha_dot=0, Omega_R=0, Omega_L=0)
    u_1 = ControlInput(0.3, [0.2, 0], [0.35, 0], 3, 65, [0, 0, 0])  # Example values, adjust as needed
    h_initial = s_0.z  # Example value, adjust as needed

    plant = Plant()

    # Get the symbolic trajectory for the stance phase
    symbolic_trajectory_stance = plant.stance_dynamics(t_symbolic, s_0, u_1, h_initial)

    # Extracting p0 and pT values from control input
    p0_x, p0_y = u_1.p_0
    pT_x, pT_y = u_1.p_T

    condition_trajectory_stance = (t_symbolic >= 0) & (t_symbolic <= u_1.T)

    # Evaluate numeric values for stance phase
    t_values_stance = np.linspace(0, float(u_1.T), 100)
    c_t_z_values_stance = compute_numeric_values(symbolic_trajectory_stance, t_values_stance, condition_trajectory_stance)

    # Plot only the z-values for the stance phase
    plot_z_values(t_values_stance, c_t_z_values_stance, 'Stance', 'blue')

if __name__ == "__main__":
    main()
