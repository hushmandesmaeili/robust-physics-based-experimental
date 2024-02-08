import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from preview_locomotion import PreviewControl, TypeOfPhases

from state import ReducedBodyTrajectory

class COMTrajectoryPlotter:
    def __init__(self, trajectory: ReducedBodyTrajectory, control: PreviewControl):
        self.trajectory = trajectory
        self.control = control

    def plot_3d_com_trajectory(self):
        # Extract COM positions
        x_com = [state.com_pos[0] for state in self.trajectory.states]
        y_com = [state.com_pos[1] for state in self.trajectory.states]
        z_com = [state.com_pos[2] for state in self.trajectory.states]

        # Extract initial and final CoP positions for all phases
        x_cop_0 = [self.control.params[i].p_0[0] for i in range(len(self.control.params)) if self.control.params[i].phase_type == TypeOfPhases.STANCE]
        y_cop_0 = [self.control.params[i].p_0[1] for i in range(len(self.control.params)) if self.control.params[i].phase_type == TypeOfPhases.STANCE]
        x_cop_T = [self.control.params[i].p_T[0] for i in range(len(self.control.params)) if self.control.params[i].phase_type == TypeOfPhases.STANCE]
        y_cop_T = [self.control.params[i].p_T[1] for i in range(len(self.control.params)) if self.control.params[i].phase_type == TypeOfPhases.STANCE]

        # Create a new 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the trajectory
        ax.plot(x_com, y_com, z_com, label='COM Trajectory')

        # Plot the CoP positions
        ax.scatter(x_cop_0, y_cop_0, 0, label='Initial CoP', color='r')
        ax.scatter(x_cop_T, y_cop_T, 0, label='Final CoP', color='g')

        # Labeling
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title('3D COM Trajectory')
        ax.legend()

        # Show the plot
        plt.show()

