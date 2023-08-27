import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the length of each link
L1 = 2
L2 = 2

# Define the joint angles for the pick and place motion
theta1 = np.linspace(0, np.pi, 100)
theta2 = np.linspace(0, np.pi, 100)

# Define the x, y, and z coordinates of the end effector
x = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
y = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
z = np.zeros_like(theta1)

# Create a 3D plot
# fig, ax = plt.subplots(nrows=2, ncols=1, gridspec_kw={'height_ratios': [3, 1]})
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-4, 4)
ax.set_ylim3d(-4, 4)
ax.set_zlim3d(0, 4)

# Define the links of the robot arm
link1, = ax.plot([], [], [], lw=2)
link2, = ax.plot([], [], [], lw=2)

# Define the end effector of the robot arm
end_effector, = ax.plot([], [], [], 'o', markersize=10)

# Define the update function for the animation
def update(i):
    # Calculate the x, y, and z coordinates of the end effector
    x_i = x[i]
    y_i = y[i]
    z_i = z[i]

    # Set the data for the links of the robot arm
    link1.set_data([0, L1 * np.sin(theta1[i])], [0, L1 * np.cos(theta1[i])])
    link1.set_3d_properties([0, 0])
    link2.set_data([L1 * np.sin(theta1[i]), L1 * np.sin(theta1[i]) + L2 * np.sin(theta1[i] + theta2[i])],
                   [L1 * np.cos(theta1[i]), L1 * np.cos(theta1[i]) + L2 * np.cos(theta1[i] + theta2[i])])
    link2.set_3d_properties([0, 0])

    # Set the data for the end effector of the robot arm
    end_effector.set_data([L1 * np.sin(theta1[i]) + L2 * np.sin(theta1[i] + theta2[i])],
                          [L1 * np.cos(theta1[i]) + L2 * np.cos(theta1[i] + theta2[i])])
    end_effector.set_3d_properties([z_i])

    # Set the title of the plot
    ax.set_title(f'Time: {i}')

    # Return the links and end effector of the robot arm
    return link1, link2, end_effector

# Create the animation
anim = FuncAnimation(fig, update, frames=len(theta1), interval=50, blit=True)

# Show the animation
plt.show()

