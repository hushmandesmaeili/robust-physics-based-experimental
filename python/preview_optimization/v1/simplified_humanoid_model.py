import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Simple3DHumanoidModel:
    def __init__(self):
        # Define link lengths
        self.torso_length = 0.5
        self.pelvis_length = 0.2
        self.hip_length = 0.3
        self.ankle_length = 0.2

        # Define foot as a quadrilateral with 3D vertices
        self.foot_vertices = [
            [0.1, -0.1, 0],
            [0.1, 0.1, 0],
            [-0.1, 0.1, 0],
            [-0.1, -0.1, 0]
        ]

        # Initialize 3D positions
        self.position = {
            'torso': [0, 0, 0],
            'pelvis': [0, 0, -self.torso_length],
            'hip1': [0.1, 0, -self.torso_length - self.pelvis_length],
            'hip2': [-0.1, 0, -self.torso_length - self.pelvis_length],
            'ankle1': [0.1, 0, -self.torso_length - self.pelvis_length - self.hip_length],
            'ankle2': [-0.1, 0, -self.torso_length - self.pelvis_length - self.hip_length],
            'foot1': [0.1, 0, -self.torso_length - self.pelvis_length - self.hip_length - self.ankle_length],
            'foot2': [-0.1, 0, -self.torso_length - self.pelvis_length - self.hip_length - self.ankle_length],
        }

        # Calculate center of mass (COM) position
        self.com_position = self.calculate_com_position()

    def calculate_com_position(self):
        # For simplicity, assuming equal mass distribution along the links
        total_mass = 4.0  # Adjust total mass as needed
        com_x = (self.position['torso'][0] + self.position['pelvis'][0] + self.position['hip1'][0] + self.position['hip2'][0] +
                 self.position['ankle1'][0] + self.position['ankle2'][0] +
                 self.position['foot1'][0] + self.position['foot2'][0]) / 8.0
        com_y = (self.position['torso'][1] + self.position['pelvis'][1] + self.position['hip1'][1] + self.position['hip2'][1] +
                 self.position['ankle1'][1] + self.position['ankle2'][1] +
                 self.position['foot1'][1] + self.position['foot2'][1]) / 8.0
        com_z = (self.position['torso'][2] + self.position['pelvis'][2] + self.position['hip1'][2] + self.position['hip2'][2] +
                 self.position['ankle1'][2] + self.position['ankle2'][2] +
                 self.position['foot1'][2] + self.position['foot2'][2]) / 8.0
        return [com_x, com_y, com_z]

    def plot_model(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot links
        ax.plot([self.position['torso'][0], self.position['pelvis'][0]],
                [self.position['torso'][1], self.position['pelvis'][1]],
                [self.position['torso'][2], self.position['pelvis'][2]], label='Torso-Pelvis', marker='o')
        ax.plot([self.position['pelvis'][0], self.position['hip1'][0]],
                [self.position['pelvis'][1], self.position['hip1'][1]],
                [self.position['pelvis'][2], self.position['hip1'][2]], label='Pelvis-Hip1', marker='o')
        ax.plot([self.position['pelvis'][0], self.position['hip2'][0]],
                [self.position['pelvis'][1], self.position['hip2'][1]],
                [self.position['pelvis'][2], self.position['hip2'][2]], label='Pelvis-Hip2', marker='o')
        ax.plot([self.position['hip1'][0], self.position['ankle1'][0]],
                [self.position['hip1'][1], self.position['ankle1'][1]],
                [self.position['hip1'][2], self.position['ankle1'][2]], label='Hip1-Ankle1', marker='o')
        ax.plot([self.position['hip2'][0], self.position['ankle2'][0]],
                [self.position['hip2'][1], self.position['ankle2'][1]],
                [self.position['hip2'][2], self.position['ankle2'][2]], label='Hip2-Ankle2', marker='o')

        # Plot feet as quadrilaterals
        foot1_polygon = Poly3DCollection([self.foot_vertices], edgecolor='black', facecolors='lightgray', alpha=0.5)
        foot2_polygon = Poly3DCollection([self.foot_vertices], edgecolor='black', facecolors='lightgray', alpha=0.5)
        ax.add_collection3d(foot1_polygon)
        ax.add_collection3d(foot2_polygon)

        # Set vertices directly for each foot
        foot1_polygon._offsets3d = [[self.position['foot1'][0], self.position['foot1'][1], self.position['foot1'][2]]]
        foot2_polygon._offsets3d = [[self.position['foot2'][0], self.position['foot2'][1], self.position['foot2'][2]]]

        # Plot center of mass (COM)
        ax.scatter(self.com_position[0], self.com_position[1], self.com_position[2], color='red', marker='x', label='COM')

        # Set axis labels
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        # Set legend
        ax.legend()

        # Show the plot
        plt.show()

# Example usage
humanoid_3d = Simple3DHumanoidModel()
humanoid_3d.plot_model()
