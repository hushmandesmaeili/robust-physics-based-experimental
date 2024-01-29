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

def compute_numeric_values(symbolic_trajectory, t_values, condition_trajectory):
    # Define the piecewise functions for COM position and velocity with bounds
    c_t_x_bound = Piecewise((symbolic_trajectory.x, condition_trajectory), (0, True))
    c_t_y_bound = Piecewise((symbolic_trajectory.y, condition_trajectory), (0, True))
    c_t_z_bound = Piecewise((symbolic_trajectory.z, condition_trajectory), (0, True))
    c_dot_t_x_bound = Piecewise((symbolic_trajectory.x_dot, condition_trajectory), (0, True))
    c_dot_t_y_bound = Piecewise((symbolic_trajectory.y_dot, condition_trajectory), (0, True))
    c_dot_t_z_bound = Piecewise((symbolic_trajectory.z_dot, condition_trajectory), (0, True))

    # Convert piecewise symbolic expressions to numeric functions
    c_t_x_bound_numeric = lambdify(t_symbolic, c_t_x_bound, 'numpy')
    c_t_y_bound_numeric = lambdify(t_symbolic, c_t_y_bound, 'numpy')
    c_t_z_bound_numeric = lambdify(t_symbolic, c_t_z_bound, 'numpy')
    c_dot_t_x_bound_numeric = lambdify(t_symbolic, c_dot_t_x_bound, 'numpy')
    c_dot_t_y_bound_numeric = lambdify(t_symbolic, c_dot_t_y_bound, 'numpy')
    c_dot_t_z_bound_numeric = lambdify(t_symbolic, c_dot_t_z_bound, 'numpy')

    # Evaluate numeric functions
    c_t_x_values_bound = c_t_x_bound_numeric(t_values) if not c_t_x_bound.equals(0) else np.zeros_like(t_values)
    c_t_y_values_bound = c_t_y_bound_numeric(t_values) if not c_t_y_bound.equals(0) else np.zeros_like(t_values)
    c_t_z_values_bound = c_t_z_bound_numeric(t_values) if not c_t_z_bound.equals(0) else np.zeros_like(t_values)
    c_dot_t_x_values_bound = c_dot_t_x_bound_numeric(t_values) if not c_dot_t_x_bound.equals(0) else np.zeros_like(t_values)
    c_dot_t_y_values_bound = c_dot_t_y_bound_numeric(t_values) if not c_dot_t_y_bound.equals(0) else np.zeros_like(t_values)
    c_dot_t_z_values_bound = c_dot_t_z_bound_numeric(t_values) if not c_dot_t_z_bound.equals(0) else np.zeros_like(t_values)

    return c_t_x_values_bound, c_t_y_values_bound, c_t_z_values_bound, \
           c_dot_t_x_values_bound, c_dot_t_y_values_bound, c_dot_t_z_values_bound

def main():
    # Define initial state (s_0), control parameters (u_1), and initial height (h_initial)
    s_0 = StateVector(x=0, y=0, z=1.7, x_dot=1.5, y_dot=0, z_dot=1.75, alpha=0, alpha_dot=0, Omega_R=0, Omega_L=0)
    u_1 = ControlInput(0.4, [0.2, 0], [0.35, 0], 1.75, 1.75, [0, 0, 0])  # Example values, adjust as needed
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
    c_t_x_values_stance, c_t_y_values_stance, c_t_z_values_stance, \
    c_dot_t_x_values_stance, c_dot_t_y_values_stance, c_dot_t_z_values_stance = compute_numeric_values(
        symbolic_trajectory_stance, t_values_stance, condition_trajectory_stance
    )

    # PHASE 2: Double-Stance
    # Use u_2 for the second phase
    u_2 = ControlInput(0.15, [float(pT_x), float(pT_y)], [pT_x + 0.75, 0], 1.75, 50, [0, 0, 0])

    # Define initial state for the second phase (s_1)
    s_1 = StateVector(c_t_x_values_stance[-1], c_t_y_values_stance[-1], c_t_z_values_stance[-1],
                      c_dot_t_x_values_stance[-1], c_dot_t_y_values_stance[-1], c_dot_t_z_values_stance[-1],
                      0, 0, 0, 0)
    # s_1 = StateVector(c_t_x_values_stance[-1], c_t_y_values_stance[-1], c_t_z_values_stance[-1],
    #                   1.5, 0, 1.75,
    #                   0, 0, 0, 0)
    
    print(c_t_z_values_stance[-1])
    print(vars(u_2), vars(s_1))

    # Get the symbolic trajectory for the double-stance phase
    symbolic_trajectory_double_stance = plant.stance_dynamics(t_symbolic, s_1, u_2, s_1.z)

    # Extracting p0 and pT values from control input for the second phase
    p0_x_2, p0_y_2 = u_2.p_0
    pT_x_2, pT_y_2 = u_2.p_T

    condition_trajectory_double_stance = (t_symbolic >= 0) & (t_symbolic <= u_2.T)

    # Evaluate numeric values for double-stance phase
    t_values_double_stance = np.linspace(0, float(u_2.T), 100)
    c_t_x_values_double_stance, c_t_y_values_double_stance, c_t_z_values_double_stance, \
    c_dot_t_x_values_double_stance, c_dot_t_y_values_double_stance, c_dot_t_z_values_double_stance = compute_numeric_values(
        symbolic_trajectory_double_stance, t_values_double_stance, condition_trajectory_double_stance
    )


    # PHASE 3: Single-Stance Again
    # Use u_3 for the third phase
    u_3 = ControlInput(0.4, [float(pT_x_2), float(pT_y_2)], [pT_x_2 + 0.15, 0], 1.75, 1.75, [0, 0, 0])

    # Define initial state for the third phase (s_2)
    s_2 = StateVector(c_t_x_values_double_stance[-1], c_t_y_values_double_stance[-1], c_t_z_values_double_stance[-1],
                    c_dot_t_x_values_double_stance[-1], c_dot_t_y_values_double_stance[-1], c_dot_t_z_values_double_stance[-1],
                    0, 0, 0, 0)

    ## Get the symbolic trajectory for the single-stance 2 phase
    symbolic_trajectory_single_stance_2 = plant.stance_dynamics(t_symbolic, s_2, u_3, s_2.z)

    # Extracting p0 and pT values from control input for the third phase
    p0_x_single_stance_2, p0_y_single_stance_2 = u_3.p_0
    pT_x_single_stance_2, pT_y_single_stance_2 = u_3.p_T

    condition_trajectory_single_stance_2 = (t_symbolic >= 0) & (t_symbolic <= u_3.T)

    # Evaluate numeric values for single-stance 2 phase
    t_values_single_stance_2 = np.linspace(0, float(u_3.T), 100)
    c_t_x_values_single_stance_2, c_t_y_values_single_stance_2, c_t_z_values_single_stance_2, \
    c_dot_t_x_values_single_stance_2, c_dot_t_y_values_single_stance_2, c_dot_t_z_values_single_stance_2 = compute_numeric_values(
        symbolic_trajectory_single_stance_2, t_values_single_stance_2, condition_trajectory_single_stance_2
    )

    # Plot both phases in a single 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # PLOT PHASE 1: single-stance phase
    ax.plot(c_t_x_values_stance, c_t_y_values_stance, c_t_z_values_stance, label='Stance Phase (c(t))', color='blue')
    # ax.plot(c_dot_t_x_values_stance, c_dot_t_y_values_stance, c_dot_t_z_values_stance,
    #         label='Stance Phase (c_dot(t))', linestyle='dashed', color='blue')
    ax.scatter([float(p0_x)], [float(p0_y)], [float(0)], color='red', label='COP Initial Position (p0)')
    ax.scatter([float(pT_x)], [float(pT_y)], [float(0)], color='green', label='COP Final Position (pT)')

    # PLOT PHASE 2: double-stance phase
    ax.plot(c_t_x_values_double_stance, c_t_y_values_double_stance, c_t_z_values_double_stance,
            label='Double-Stance Phase (c(t))', color='orange')
    # ax.plot(c_dot_t_x_values_double_stance, c_dot_t_y_values_double_stance, c_dot_t_z_values_double_stance,
    #         label='Double-Stance Phase (c_dot(t))', linestyle='dashed', color='orange')
    ax.scatter([float(p0_x_2)], [float(p0_y_2)], [float(0)], color='purple', label='COP Initial Position (p0_2)')
    ax.scatter([float(pT_x_2)], [float(pT_y_2)], [float(0)], color='brown', label='COP Final Position (pT_2)')


    #PLOT PHASE 3: singe-stance phase
    # Plot triple-stance phase
    # ax.plot(c_t_x_values_single_stance_2, c_t_y_values_single_stance_2, c_t_z_values_single_stance_2,
    #     label='Single-Stance 2 Phase (c(t))', color='purple')
    # ax.scatter([float(p0_x_single_stance_2)], [float(p0_y_single_stance_2)], [float(0)],
    #         color='orange', label='COP Initial Position (p0_single_stance_2)')
    # ax.scatter([float(pT_x_single_stance_2)], [float(pT_y_single_stance_2)], [float(0)],
    #         color='pink', label='COP Final Position (pT_single_stance_2)')

    # # Set bounds for the plot
    # ax.set_xlim([0, 2])  # Adjust the x-axis bounds as needed
    # ax.set_ylim([0, 2])  # Adjust the y-axis bounds as needed
    # ax.set_zlim([0, 3])  # Adjust the z-axis bounds as needed

    # Customize the 3D plot
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Combined Stance and Double-Stance Phases (3D)')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    main()
