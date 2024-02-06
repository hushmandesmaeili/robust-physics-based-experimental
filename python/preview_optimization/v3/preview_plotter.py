import matplotlib.pyplot as plt

class PreviewPlotter:
    def __init__(self):
        pass

    def plot_preview(self, preview, optional_plots=[]):
        fig, ax = plt.subplots()
        ax.plot(preview['com_position'][0], preview['com_position'][1], label='COM Position')
        ax.scatter([preview['initial_cop'][0], preview['final_cop'][0]], 
                   [preview['initial_cop'][1], preview['final_cop'][1]], 
                   color='red', label='COP')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.legend()

        if 'z' in optional_plots:
            fig_z, ax_z = plt.subplots()
            ax_z.plot(preview['com_position'][2], label='z(t)')
            ax_z.set_xlabel('t')
            ax_z.set_ylabel('z')
            ax_z.legend()

        if 'com_velocity' in optional_plots:
            fig_v, ax_v = plt.subplots()
            ax_v.plot(preview['com_velocity'], label='COM Velocity')
            ax_v.set_xlabel('t')
            ax_v.set_ylabel('Velocity')
            ax_v.legend()

        plt.show()