from resonance.linear_systems import SingleDoFLinearSystem
import matplotlib.pyplot as plt
import numpy as np
from resonance.functions import spring
import matplotlib.animation as animation


sys = SingleDoFLinearSystem()

sys.constants['m'] = 7.0  # kg
sys.constants['fn'] = 1.0  # Hz
sys.constants['zeta'] = 0.2  # unitless



sys.coordinates['x'] = 1.0  # m
sys.speeds['v'] = 0.0  # m/s



def calculate_canonical_coefficients(m, fn, zeta):
    wn = 2*np.pi*fn
    k = m*wn**2
    c = zeta*2*wn*m
    return m, c, k


sys.canonical_coeffs_func = calculate_canonical_coefficients

sys.canonical_coefficients()

sys.period()

sys

sys.coordinates['x'] = -5.0
sys.speeds['v'] = 8.0

# under-damped
trajectories = sys.free_response(5.0)
axes = trajectories.plot(subplots=True)
plt.title('under-damped')
plt.show()

# #undamped
sys.constants['zeta'] = 0
trajectories = sys.free_response(5)
axes = trajectories.plot(subplots=True)
plt.title('undamped')
plt.show()
#
# #critically-damped
sys.constants['zeta'] = 1
trajectories = sys.free_response(5)
axes = trajectories.plot(subplots=True)
plt.title('critically-damped')
plt.show()
#
# #over-damped
sys.constants['zeta'] = 2.5
trajectories = sys.free_response(5)
axes = trajectories.plot(subplots=True)
plt.title('over-damped')
plt.show()

# sys.constants['l'] = 0.2  # m
# def create_configuration_figure(x, l):
#
#     # create a figure with one or more axes
#     fig, ax = plt.subplots()
#
#     # the `spring()` function creates the x and y data for plotting a simple
#     # spring
#     spring_x_data, spring_y_data = spring(0.0, x, l/2, l/2, l/8, n=3)
#     lines = ax.plot(spring_x_data, spring_y_data, color='purple')
#     spring_line = lines[0]
#
#     # add a square that represents the mass
#     square = plt.Rectangle((x, 0.0), width=l, height=l, color='blue')
#     ax.add_patch(square)
#
#     # add a vertical line representing the spring's attachment point
#     ax.axvline(0.0, linewidth=4.0, color='black')
#
#     # set axis limits and aspect ratio such that the entire motion will appear
#     ax.set_ylim((-l/2, 3*l/2))
#     ax.set_xlim((-np.abs(x) - l, np.abs(x) + l))
#     ax.set_aspect('equal')
#
#     ax.set_xlabel('$x$ [m]')
#
#     # this function must return the figure as the first item
#     # but you also may return any number of objects that you'd like to have
#     # access to modify, e.g. for an animation update
#
#     return fig, ax, spring_line, square
#
# # associate the function with the system
# sys.config_plot_func = create_configuration_figure
#
# sys.constants['zeta'] = 0.2
# trajectories = sys.free_response(5.0)
#
# def update_configuration(x, l, time,  # any variables you need for updating
#                          ax, spring_line, square):  # returned items from plot_configuration() in same order
#
#     ax.set_title('{:1.2f} [s]'.format(time))
#
#     xs, ys = spring(0.0, x, l/2, l/2, l/8, n=3)
#     spring_line.set_data(xs, ys)
#
#     square.set_xy((x, 0.0))
#
# sys.config_plot_update_func = update_configuration
# animation = sys.animate_configuration(fps=30)
# plt.show()


# # Define the figure and axis
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import numpy as np
#
# fig, ax = plt.subplots()
#
# # Increase the range of the x and y axes
# ax.set_xlim(-6, 6)
# ax.set_ylim(-6, 6)
#
# robot = plt.Circle((0, 0), radius=0.5, fc='blue')
# ax.add_artist(robot)
#
#
# def animate(frame):
#     # Define the center of the circle and the radius
#     center = np.array([0, 0])
#     radius = 5
#
#     # Calculate the new position of the robot using the equation for a circle
#     angle = 2 * np.pi * frame / 100
#     x = center[0] + radius * np.cos(angle)
#     y = center[1] + radius * np.sin(angle)
#     robot.center = (x, y)
#
#     return [robot]
#
#
# ani = animation.FuncAnimation(fig, animate, frames=100, interval=50, blit=True)
#
# # Increase the size of the figure
# fig.set_size_inches(8, 8)
#
# # Show the animation
# plt.show()