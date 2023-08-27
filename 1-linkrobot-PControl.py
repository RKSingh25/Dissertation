import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define constants
m = 2  # Mass of the link
l = 1.0  # Length of the link
g = 9.81  # Acceleration due to gravity
k_p = 12
c = 1

# Define simulation parameters
dt = 0.01  # Time step
t = np.arange(0.0, 10.0, dt)  # Time array

# Define initial conditions
theta = 0.0  # Joint angle
theta_dot = 0.0  # Joint velocity

# Define reference signal
r = np.ones(len(t))

# Initialize arrays to store results
theta_arr = np.zeros(len(t))
theta_dot_arr = np.zeros(len(t))
e_arr = np.zeros(len(t))

# Set up the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
# ax1.set_xlim(0, t[-1])
# ax1.set_ylim(-2, 2)
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
    global theta, theta_dot, error_integral, counter, error_integral_vals, time_vals, theta_va,diff

    # Compute joint acceleration
    # theta_ddot = (k_p/(m*l**2))*(r[frame] - theta - ((m*g*l*np.sin(theta))/(2*k_p))) # gravity no damping
    # theta_ddot = (k_p / (m * l ** 2)) * (r[frame] - theta) #no gravity no damping
    theta_ddot = (k_p/(m*l**2))*(r[frame] - theta - ((m*g*l*np.sin(theta))/(2*k_p)) - ((c/k_p)*theta_dot)) #gravity and damping

    # Update joint velocity and angle
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # Compute end-effector position
    x = l * np.sin(theta)
    y = -l * np.cos(theta)

    # Update line data
    line.set_data([0, x], [0, y])

    # Compute error between the reference signal and the joint angle
    e = - theta + r[frame]
    e_arr[frame] = e
    error_vals.append(e)
    u = k_p*e
    # print(f"Time: {t[frame]:.2f} s, Reference Signal: {r[frame]:.2f}, Joint Angle: {theta:.2f}, Error: {e:.2f}")
    # Compute integral of error using trapezoidal rule
    # if counter == integral_interval:
    #     error_integral += (e + error_arr[frame - integral_interval]) * dt / 2.0
    #     error_integral_arr[frame] = error_integral
    #     counter = 0
    # else:
    #     error_integral_arr[frame] = error_integral_arr[frame - 1]
    #     counter += 1
    # error_integral_vals.append(error_integral)
    time_vals.append(t[frame])
    reference.append(r[frame])
    theta_va.append(theta)
    u_arr.append(u)
    # Plot error vs. time
    # ax1.clear()
    # ax1.plot(t[:frame], e_arr[:frame])
    # ax1.set_xlabel('Time (s)')
    # ax1.set_ylabel('Error')
    # ax1.set_title('Error between Joint Angle and Reference Signal')

    # Plot error and error integral vs. time
    # ax1.clear()
    # ax1.plot(t[:frame], error_arr[:frame])
    # ax1.set_xlabel('Time (s)')
    # ax1.set_ylabel('Error')
    # ax1.set_title('Error between Joint Angle and Reference Signal')
    # ax1.clear()
    # ax1.plot(t[:frame], error_integral_arr[:frame])
    # ax1.set_xlabel('Time (s)')
    # ax1.set_ylabel('Error Integral')
    # ax1.set_title('Integral of Error between Joint Angle and Reference Signal')
    # print(error_integral_arr)

    # Update error plot data at every frame
    line2.set_data(time_vals, error_vals)
    ax1.relim()
    ax1.autoscale_view()
    # Update error integral plot data at every integral interval
    # line2.set_data(time_vals, error_integral_vals)
    # ax1.relim()
    # ax1.autoscale_view()
    # line2.set_data(time_vals, theta_va)
    # ax1.relim()
    # ax1.autoscale_view()

    # Update theta vs time plot data at every frame
    ax2.plot(time_vals, theta_va, color='blue')
    ax2.relim()
    ax2.autoscale_view()

    return line, line2

# Create the animation
anim = FuncAnimation(fig, update, frames=len(t), interval=dt * 1000, blit=True,repeat = False)


plt.show()
# print(error_vals)

# fig2,ax = plt.subplots()
# ax.plot(time_vals, error_integral_vals)
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('Error')
# ax.set_title('Integral of Error between the Reference Signal and Joint Angle')
# plt.show()
#
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
print(theta_va)
print(t)
# print(reference)