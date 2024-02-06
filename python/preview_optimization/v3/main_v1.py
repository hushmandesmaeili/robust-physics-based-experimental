import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from state import State
from model_control_params import ModelControlParams
from preview_locomotion import PreviewLocomotion
from preview_optimization import PreviewOptimization

def main():
    c_0 = [0, 0, 1.7]
    c_dot_0 = [1.5, 0, 1.5]
    initial_state = State(c_0, c_dot_0, 0, 0, 0, 0)

    phase_duration, p_0, p_T, r_0, r_T, alpha_ddot = 0.4, [0.2, 0], [0.35, 0], 1.75, 1.75, 0
    control_input = ModelControlParams(phase_duration, p_0, p_T, r_0, r_T, alpha_ddot)

    preview_locomotion = PreviewLocomotion()
    stance_preview = preview_locomotion.generate_stance_preview(initial_state, control_input)

    # Generate numerical values for stance_preview.x
    t_values = np.linspace(0, control_input.T_duration, num=100)  # Adjust the number of points as needed
    t = sp.symbols('t')  # Symbolic time variable

    # Use lambdify to create numpy-compatible functions
    x_func = sp.lambdify(t, stance_preview.x, modules=['numpy']) 
    y_func = sp.lambdify(t, stance_preview.y, modules=['numpy'])
    z_func = sp.lambdify(t, stance_preview.z, modules=['numpy']) 

    # Evaluate the functions over the array of time values
    x_values = x_func(t_values) if not stance_preview.x.equals(0) else np.zeros_like(t_values)
    y_values = y_func(t_values) if not stance_preview.y.equals(0) else np.zeros_like(t_values)
    z_values = z_func(t_values) if not stance_preview.z.equals(0) else np.zeros_like(t_values)

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_values, y_values, z_values, label='x, y, z')
    ax.scatter(p_0[0], p_0[1], color='red', label='p_0')
    ax.scatter(p_T[0], p_T[1], color='green', label='p_T')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()

if __name__ == "__main__":
    main()