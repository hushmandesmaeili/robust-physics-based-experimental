from sympy import symbols, Piecewise, lambdify
from python.preview_optimization.state_vector import StateVector
from control_input import ControlInput
from plant import Plant

class PreviewSimulation:
    def __init__(self, t):
        self.t = t

    def generate_preview_simulation(self, initial_state, control_parameters):
        individual_trajectories = []

        for i in range(0, len(control_parameters), 6):
            # Extract control parameters for the current phase
            current_control_input = ControlInput(*control_parameters[i:i+6])

            # Create a new plant for each phase (modify as needed)
            plant = Plant(plant_parameters)  # Add relevant plant initialization logic

            # Generate the symbolic trajectory for the current phase using stance_dynamics
            current_trajectory = self._generate_symbolic_phase_trajectory(
                plant, initial_state, current_control_input)

            # Append the trajectory to the list
            individual_trajectories.append(current_trajectory)

        # Concatenate the individual symbolic trajectories to form the preview simulation
        preview_simulation = self._concatenate_symbolic_trajectories(
            individual_trajectories)

        return preview_simulation

    def _generate_symbolic_phase_trajectory(self, plant, initial_conditions, control_input):
        symbolic_trajectory = []

        for t in range(0, control_input.T):
            # Call plant.stance_dynamics to get the symbolic state at each time step
            current_state = plant.stance_dynamics(
                self.t, initial_conditions, control_input, h_initial)

            symbolic_trajectory.append(current_state)

        return symbolic_trajectory

    def _concatenate_symbolic_trajectories(self, symbolic_trajectories):
        # Concatenate the symbolic trajectories using Piecewise
        concatenated_trajectory = Piecewise(*symbolic_trajectories)

        return concatenated_trajectory

# Example usage
t_symbolic = symbols('t')
preview_simulation = PreviewSimulation(t_symbolic)

# Initial state vector (modify as needed)
initial_state = StateVector(x=0, y=0, z=0, x_dot=0, y_dot=0, z_dot=0, alpha=0, alpha_dot=0)

# Example control parameters (modify as needed)
control_parameters = [T1, p0_1, pT_1, r0_1, rT_1, alpha_ddot_1,
                      T2, p0_2, pT_2, r0_2, rT_2, alpha_ddot_2,
                      T3, p0_3, pT_3, r0_3, rT_3, alpha_ddot_3]

symbolic_preview_simulation = preview_simulation.generate_preview_simulation(
    initial_state, control_parameters)

# Convert the symbolic preview simulation to a lambda function if needed
numeric_preview_simulation = lambdify(t_symbolic, symbolic_preview_simulation)

# Now you can use symbolic_preview_simulation or numeric_preview_simulation as needed
