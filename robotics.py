import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


# Define the robot parameters
L1 = 1
L2 = 1

# Create the figure and axes
# fig, ax = plt.subplots()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
fig, ax = plt.subplots(nrows=2, ncols=1, gridspec_kw={'height_ratios': [3, 1]})
ax[0].set_xlim(-2.5, 2.5)
ax[0].set_ylim(-2.5, 2.5)


# Define the links of the robot arm
link1, = ax[0].plot([], [], lw=2)
link2, = ax[0].plot([], [], lw=2)


# Define the robot base and end effector
base = plt.Circle((0, 0), 0.1, fc='b')
ee = plt.Circle((0, 0), 0.1, fc='r')
ax[0].add_artist(base)
ax[0].add_artist(ee)

# Define the initial configuration of the robot arm
theta1 = np.pi / 4
theta2 = np.pi / 4

timer_text = ax[0].text(0.05, 0.95, '', transform=ax[0].transAxes)

# Define the forward kinematics function
def forward_kinematics(theta1, theta2):
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

theta1_values = []
theta2_values = []
time_values = []
fps = 20
total_frames = 2000000
# Define the update function for the animation
def update(frame):
    global theta1, theta2
    # Update the joint angles based on the frame number
    theta1 = np.sin(frame / 50 * 2 * np.pi) * np.pi / 4 + np.pi / 4
    theta2 = np.sin(frame / 50 * 2 * np.pi) * np.pi / 4

    # Compute the forward kinematics
    x, y = forward_kinematics(theta1, theta2)

    # Update the links and end effector positions
    link1.set_data([0, L1 * np.cos(theta1)], [0, L1 * np.sin(theta1)])
    link2.set_data([L1 * np.cos(theta1), x], [L1 * np.sin(theta1), y])

    ee.center = (x, y)
    elapsed_time = frame / fps  # Calculate the elapsed time
    timer_text.set_text(f'Time: {elapsed_time:.1f}s')

    #calculate time, theta1 and theta2 values
    current_time = frame / fps
    theta1_values.append(theta1)
    theta2_values.append(theta2)
    time_values.append(current_time)
    theta1dot = np.diff(theta1_values) / np.diff(time_values)

    # ax[1].clear()
    # ax[1].plot(time_values, theta1_values)
    # ax[1].set_xlabel('Time (s)')
    # ax[1].set_ylabel(r'$\theta_{1}$ (rad)')

    # ax[2].clear()
    # ax[2].plot(time_values, theta2_values)
    # ax[2].set_xlabel('Time (s)')
    # ax[2].set_ylabel(r'$\theta_{2}$ (rad)')

    ax[1].clear()
    ax[1].plot(time_values[:-1], theta1dot)
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel(''r'$\omega_{1}$ (rad/s)')

    # ax[1].clear()
    # ax[1].plot(theta1_values[:-1], theta1dot)
    # ax[1].set_xlabel(r'$\theta_{1}$ (rad)')
    # ax[1].set_ylabel(''r'$\omega_{1}$ (rad/s)')


    return link1, link2, base, ee, timer_text


# Create the animation object
ani = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)
# ani.event_source.start()
# plt.show()

# Define the buttons and their callbacks
def start_animation(event):
    ani.event_source.start()
    plt.show()


def stop_animation(event):
    ani.event_source.stop()
    plt.show()


def reset_animation(event):
    global theta1, theta2
    ani.event_source.stop()
    link1.set_data([], [])
    link2.set_data([], [])
    ee.center = (0, 0)
    theta1 = np.pi / 4
    theta2 = np.pi / 4
    ani.event_source.start()


start_ax = plt.axes([0.1, 0.05, 0.1, 0.075])
start_button = Button(start_ax, 'Start')
start_button.on_clicked(start_animation)

stop_ax = plt.axes([0.25, 0.05, 0.1, 0.075])
stop_button = Button(stop_ax, 'Stop')
stop_button.on_clicked(stop_animation)

reset_ax = plt.axes([0.4, 0.05, 0.1, 0.075])
reset_button = Button(reset_ax, 'Reset')
reset_button.on_clicked(reset_animation)

plt.show()

plt.plot(time_values, theta2_values)
plt.xlabel('Time (s)')
plt.ylabel(''r'$\theta_{2}$ (rad)')
plt.show()

# Calculate theta1dot and plot it against time
# theta1dot = np.diff(theta1_values)/np.diff(time_values)
# plt.plot(time_values[:-1], theta1dot)
# plt.xlabel('Time')
# plt.ylabel(''r'$\omega_{1}$ (rad)')
#
# plt.show()
