import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define constants
m = 2  # Mass of the link
l = 1.0  # Length of the link
g = 9.81  # Acceleration due to gravity
k_p = 12
k_i = 10
k_d = 8
c = 3


# Define simulation parameters
dt = 0.001  # Time step
t = np.arange(0.0, 10.0, dt)  # Time array

# Define initial conditions
theta = np.pi/2  # Joint angle
theta_dot = 0.0  # Joint velocity
error_integral = 0.0 # Initialize error integral

# Define reference signal
r = np.ones(len(t))

# Initialize arrays to store results
theta_arr = np.zeros(len(t))
theta_dot_arr = np.zeros(len(t))
e_arr = np.zeros(len(t))

# Set up the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Error')
ax1.set_title('Error between Reference Signal and Joint Angle')
ax2.set_xlim(-1.1, 1.1)
ax2.set_ylim(-1.1, 1.1)
ax2.set_aspect('equal')
line, = ax2.plot([], [], 'o-', lw=2)
integral_interval = 1
counter = 0
reference = []
time_vals = []
error_vals = []
theta_va = []
u_arr = []
line2, = ax1.plot([],[],'b-', lw=2)

def update(frame):
    global theta, theta_dot, error_integral, counter, error_integral_vals, time_vals, theta_va, diff, prev_e

    # Compute joint acceleration
    e = r[frame] - theta
    error_vals.append(e)
    error_integral += e * dt
    diff = (e - prev_e) / dt if frame > 0 else 0
    theta_ddot = (1/(m*l**2))*((k_p * e + k_i * error_integral + k_d * diff) - c*theta_dot - (m*g*l*np.sin(theta)/2))
    prev_e = e

    # Update joint velocity and angle
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # Compute end-effector position
    x = l * np.cos(theta)
    y = l * np.sin(theta)

    # Update line data
    line.set_data([0, x], [0, y])

    u = (k_p * e + k_i * error_integral + k_d * diff)
    #append to lists
    u_arr.append(u)
    # Update error plot data at every frame
    time_vals.append(t[frame])
    theta_va.append(theta)
    # line2.set_data(time_vals, theta_va)
    # ax1.relim()
    # ax1.autoscale_view()
    #
    # # Update theta vs time plot data at every frame
    # ax2.plot(time_vals, theta_va, color='blue')
    # ax2.relim()
    # ax2.autoscale_view()

    return line, line2

# Create the animation
anim = FuncAnimation(fig, update, frames=len(t), interval=dt * 1000, blit=True,)

plt.show()

fig2,ax = plt.subplots()
ax.plot(time_vals, u_arr)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Controller Output')
plt.show()

fig3,ax = plt.subplots()
ax.plot(time_vals, theta_va)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Joint Angle (rad)')
plt.show()

