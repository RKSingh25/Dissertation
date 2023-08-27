import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

plt.rc('font', family='Helvetica')

# Define constants
m1 = 1.0  # Mass of link 1
m2 = 1.0  # Mass of link 2
L1 = 1.0  # Length of link 1
L2 = 1.0  # Length of link 2KC = 75, TC = 1.1
g = 9.81  # Acceleration due to gravity
Kp1 = 180 # Proportional gain for joint 1
Kp2 = 180 # Proportional gain for joint 2
Kd1 = 24 # Derivative gain for joint 1
Kd2 = 20 # Derivative gain for joint 2
Ki1 = 450 # Integral gain for joint 1 #450
Ki2 = 507 # Integral gain for joint 2 #507


T = 10 # Simulation Time
t_span = (0, T)
dt = 0.001 # Time step
t = np.arange(t_span[0], t_span[1]+dt, dt)

# Define desired trajectory for end-effector position in Cartesian space

# # Line Trajectory
# x_desired = np.linspace(-1, 1, int(len(t)))
# y_desired = 1.414*np.ones(int(len(t)))

# # Circle Trajectory
r = 0.72
omega = 2 * np.pi / T
x_desired = 1 + (r * np.cos(omega*t))
y_desired = r * np.sin(omega*t)

# # Square Trajectories

# x_desired = np.concatenate((1.5*np.ones(int(len(t)/5)), np.linspace(1.5, -0.5, int(len(t)/5)), -1*0.5*np.ones(int(len(t)/5)), np.linspace(-0.5,1.5,int(len(t)/5)),1.5*np.ones(int(len(t)/5)+1)))
# y_desired = np.concatenate((np.linspace(0,1,int(len(t)/5)), np.ones(int(len(t)/5)),np.linspace(1,-1,int(len(t)/5)), -1*np.ones(int(len(t)/5)),np.linspace(-1,0,int(len(t)/5)+1)))
# x_desired = np.concatenate((np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)), np.zeros(int(len(t)/4)+1)))
# y_desired = np.concatenate((np.zeros(int(len(t)/4)), np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)+1)))
x_desired = np.concatenate((1.7*np.ones(int(len(t)/5)), np.linspace(1.7, 0.7, int(len(t)/5)), 0.7*np.ones(int(len(t)/5)), np.linspace(0.7,1.7,int(len(t)/5)),1.7*np.ones(int(len(t)/5)+1)))
y_desired = np.concatenate((np.linspace(0,0.5,int(len(t)/5)), 0.5*np.ones(int(len(t)/5)),np.linspace(0.5,-0.5,int(len(t)/5)), -0.5*np.ones(int(len(t)/5)),np.linspace(-0.5,0,int(len(t)/5)+1)))
# theta1_desired = np.concatenate((np.linspace(-np.pi/2,0,int(len(t)/4)),np.linspace(0,-np.pi/4,int(len(t)/8)),np.linspace(-np.pi/4,1,int(len(t)/4)),np.linspace(1,-np.pi/8,int(3*(len(t)/8)+1))))
# theta2_desired = np.concatenate((np.linspace(0,np.pi/4,int(len(t)/4)),np.linspace(np.pi/4,np.pi/8,int(len(t)/4)),np.linspace(np.pi/8,np.pi/6,int(len(t)/8)),np.linspace(np.pi/6,np.pi/3,int(3*(len(t)/8)+1))))
# theta2_desired = np.concatenate((np.linspace(-np.pi/2,0,int(len(t)/4)),np.linspace(0,-np.pi/4,int(len(t)/8)),np.linspace(-np.pi/4,1,int(len(t)/4)),np.linspace(1,-np.pi/8,int(3*(len(t)/8)+1))))
theta1_desired = np.zeros_like(x_desired)
theta2_desired = np.zeros_like(x_desired)

prev_theta1d = 0
prev_theta2d = 0

def theta_desired(theta1d,theta2d):
    return theta1d,theta2d

def forward_kinematics(theta1, theta2, l1, l2):
    x = l1*np.cos(theta1) + l2*np.cos(theta1+theta2)
    y = l1*np.sin(theta1) + l2*np.sin(theta1+theta2)
    return x, y

# x_desired,y_desired = forward_kinematics(theta1_desired,theta2_desired,L1,L2)
def positive_radians(angle, prev_angle):
    if angle < 0:
        if prev_angle is None or abs(angle - prev_angle) > np.pi:
            degrees = 360 - ((angle*(360/(2*np.pi)))*(-1))
            rad = 2*np.pi*(degrees/360)
            return rad
    return angle
the1=[]
# Compute corresponding desired joint angles using inverse kinematics
for i in range(len(x_desired)):
    x = x_desired[i]
    y = y_desired[i]
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2*L1*L2)
    s2 = np.sqrt((1 - (c2**2)))

    theta2_desired[i] = np.arctan2(s2, c2)
    theta2_desired[i] = positive_radians(theta2_desired[i],prev_theta2d)
    prev_theta2d = theta2_desired[i]

    k1 = L1 + L2*c2
    k2 = L2*s2

    theta1_desired[i] = np.arctan2(y, x) - np.arctan2(k2, k1)
    theta1_desired[i] = positive_radians(theta1_desired[i],prev_theta1d)
    prev_theta1d = theta1_desired[i]

# for i in range(len(theta1_desired)):
#     theta2_desired[i] = positive_radians(theta2_desired[i], prev_theta2d)
#     prev_theta2d = theta2_desired[i]
#     theta1_desired[i] = positive_radians(theta1_desired[i], prev_theta1d)
#     prev_theta1d = theta1_desired[i]
#     the = theta_desired(theta1_desired[i], theta2_desired[i])
#     the1.append(the)
# Compute desired joint velocities
theta1dot_desired = np.diff(theta1_desired) / dt
theta1dot_desired = np.append(theta1dot_desired, theta1dot_desired[-1])
theta2dot_desired = np.diff(theta2_desired) / dt
theta2dot_desired = np.append(theta2dot_desired, theta2dot_desired[-1])

theta1dotdot_desired = np.diff(theta1dot_desired) / dt
theta1dotdot_desired = np.append(theta1dotdot_desired, theta1dotdot_desired[-1])
theta2dotdot_desired = np.diff(theta2dot_desired) / dt
theta2dotdot_desired = np.append(theta2dotdot_desired, theta2dotdot_desired[-1])
# Define initial conditions
theta1 = theta1_desired[0]  # Joint 1 angle
theta2 = theta2_desired[0]  # Joint 2 angle
theta1_dot = theta1dot_desired[0]  # Joint 1 angular velocity
theta2_dot = theta2dot_desired[0]  # Joint 2 angular velocity

def Mass_Matrix(theta2):
    J1 = (1 / 12) * m1 * L1 ** 2
    J2 = (1 / 12) * m2 * L2 ** 2
    M11 = J1 + J2 + (m1*(L1/2)**2) + (m2*((L1**2)+((L2/2)**2) + (2*L1*(L2/2)*np.cos(theta2))))
    M12 = J2 + m2*(((L2/2)**2) + (L1*(L2/2)*np.cos(theta2)))
    M21 = J2 + m2*(((L2/2)**2) + (L1*(L2/2)*np.cos(theta2)))
    M22 = J2 + m2*((L2/2)**2)
    return np.array([[M11,M12],[M21,M22]])

def Coriolis_Matrix(theta1_dot,theta2_dot,theta2):
    b = m2*L1*(L2/2)*np.sin(theta2)
    C11 = -1*b*theta2_dot
    C12 = (-1*b*theta1_dot) - (b*theta1_dot)
    C21 = (b*theta1_dot)
    C22 = 0
    return np.array([[C11,C12],[C21,C22]])

def Gravity_Vector(theta1, theta2):
    global g
    G11 = (m1*(L1/2)*np.cos(theta1)) + (m2*((L2/2)*np.cos(theta1 + theta2) + (L1*np.cos(theta1))))
    G21 = m2*(L2/2)*np.cos(theta1 + theta2)
    return np.array([[g*G11],[g*G21]])

def Theta_dot():
    return np.array([[theta1_dot], [theta2_dot]])

def Theta():
    return np.array([[theta1], [theta2]])

theta_vector = Theta()
theta_dot_vector = Theta_dot()

x1_vals = []
x2_vals = []
y1_vals = []
y2_vals = []
theta1_vals = []
theta2_vals = []
time_vals = []
trajectory = []
tau1arr = []
tau2arr = []
cumulative_error1 = 0
erl1 = []
cumulative_error2 = 0
erl2 = []
error1_prev = 0
error2_prev = 0
error1_integral = 0
error2_integral = 0
tau1c = 0
tau2c = 0
tau1l = []
tau2l = []
powerc1 = 0
powerc2 = 0
powerc1l = []
powerc2l = []
max_torque1 = 15
max_torque2 = 15
ISE1 = 0
ISE2 = 0
e1=[]
e2=[]
theta1dot_vals = []
theta2dot_vals = []
wind_torque = np.concatenate((1.5*np.zeros(int(2*(len(t)/3))), np.linspace(0, 14.38, int(len(t)/3)+1)))
t1ddot = []
t2ddot = []
start_time = time.time()

for i in range(len(t)):

    theta1dot_vals.append(theta1_dot)
    theta2dot_vals.append(theta2_dot)
    theta1_vals.append(theta1)
    theta2_vals.append(theta2)
    error_theta1 = theta1_desired[i] - theta1
    cumulative_error1 += (abs(error_theta1))*dt
    erl1.append(cumulative_error1)
    e1.append(abs(error_theta1))
    error_theta2 = theta2_desired[i] - theta2
    cumulative_error2 += (abs(error_theta2))*dt
    erl2.append(cumulative_error2)
    e2.append(abs(error_theta2))
    error1_integral += error_theta1*dt
    error2_integral += error_theta2*dt
    errord_theta1 = (error_theta1 - error1_prev)/dt
    errord_theta2 = (error_theta2 - error2_prev)/dt
    # Disturbance (medium gust of wind between 6 and 10 seconds)
    disturbance = np.zeros((2, 1))  # initialize disturbance vector
    # disturbance[0] = wind_torque[i]  # add disturbance torque to first joint
    # disturbance[1] = wind_torque[i]

    # # PID Control
    # tau1 = (Kp1*error_theta1) + (Ki1*error1_integral) + (Kd1*errord_theta1)
    # tau2 = (Kp2*error_theta2) + (Ki2*error2_integral) + (Kd2*errord_theta2)

    #PD Control
    # tau1 = (Kp1 * error_theta1) + (Kd1 * errord_theta1)
    # tau2 = (Kp2 * error_theta2) + (Kd2 * errord_theta2)
    # tau2 = 0

    # PI Control
    # tau1 = (Kp1*error_theta1) + (Ki1*error1_integral)
    # tau2 = (Kp2*error_theta2) + (Ki2*error2_integral)

    # # #P Control
    tau1 = (Kp1 * error_theta1)
    tau2 = (Kp2 * error_theta2)

    ## Motor Saturation
    # if tau1 >= limit:
    #     tau1 = limit
    #
    # if tau2 >= limit:
    #     tau2 = limit
    tau1 = np.clip(tau1, -max_torque1, max_torque1)
    tau2 = np.clip(tau2, -max_torque2, max_torque2)


    tau1arr.append(cumulative_error1)
    tau2arr.append(cumulative_error2)
    invM = np.linalg.inv(Mass_Matrix(theta2))
    C = Coriolis_Matrix(theta1_dot, theta2_dot, theta2)
    G = Gravity_Vector(theta1, theta2)
    tau_vector = np.array([[tau1], [tau2]])
    tau_vector_disturbed = tau_vector + disturbance
    thetad_dot_vector = invM @ (tau_vector_disturbed - C@theta_dot_vector - G)
    t1ddot.append(thetad_dot_vector[0][0])# thetad_dot_vector = invM @ (tau_vector)
    t2ddot.append(thetad_dot_vector[1][0])
    theta1_dot += thetad_dot_vector[0][0]*dt
    theta2_dot += thetad_dot_vector[1][0]*dt
    tau1c += abs(tau1)*dt
    tau2c += abs(tau2)*dt
    tau1l.append(tau1c)
    tau2l.append(tau2c)
    powerc1 += abs(tau1*theta1_dot)
    powerc2 += abs(tau2*theta2_dot)
    powerc1l.append(powerc1)
    powerc2l.append(powerc2)
    theta1 += theta1_dot*dt
    theta2 += theta2_dot*dt
    error1_prev = error_theta1
    error2_prev = error_theta2
    time_vals.append(t[i])

    # Update position of links in animation
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + (L2 * np.cos(theta1 + theta2))
    y2 = y1 + (L2 * np.sin(theta1 + theta2))
    x1_vals.append(x1)
    x2_vals.append(x2)
    y1_vals.append(y1)
    y2_vals.append(y2)


end_time = time.time()
time_elapsed = end_time - start_time

def update(frame,line,trajectory_line,):

    # Get the precomputed values for the given frame
    x1, y1, x2, y2 = x1_vals[frame], y1_vals[frame], x2_vals[frame], y2_vals[frame]

    # Update the position of the links
    line.set_data([0, x1, x2], [0, y1, y2])
    trajectory.append([x2, y2])

    # Plot the trajectory
    x_traj, y_traj = zip(*trajectory)
    trajectory_line.set_data(x_traj, y_traj)

    return line, trajectory_line,

# Initialise a figure for the live simulation
fig, ax = plt.subplots()
# ax.set_title('2 link robot animation with proportional integral derivative control')
ax.plot(x_desired, y_desired, label='Desired Trajectory')
ax.set_xlim(-2.1, 2.1)
ax.set_ylim(-2.1, 2.4)
ax.set_aspect('equal')
ax.set_xlabel('x (m)', fontsize = 14)
ax.set_ylabel('y (m)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
line, = ax.plot([], [], 'o-', lw=2)
trajectory_line, = ax.plot([], [], 'r--', lw=2, label='Actual Trajectory')
plt.legend(loc='upper right', fontsize = 12.5)


# Create the animation
frames = range(0, len(t), 11)
ani = FuncAnimation(fig, update, frames=frames, fargs=(line,trajectory_line,), interval=1000*dt, blit=True, repeat=False)

# Display the animation
plt.show()

# PLOTS
tau1_array = np.array(tau1arr)
tau2_array = np.array(tau2arr)
np.savetxt('tauvalues_line_P.txt', np.column_stack((tau1_array, tau2_array)), delimiter=',')
x1_array = np.array(x1_vals)
x2_array = np.array(x2_vals)
y1_array = np.array(y1_vals)
y2_array = np.array(y2_vals)
np.savetxt('cartesian_values_P.txt', np.column_stack((x1_array, x2_array,y1_array,y2_array)), delimiter=',')
# theta1 desired and theta1 actual vs time
fig,ax = plt.subplots()
ax.plot(time_vals, theta1_vals, label = r'$\theta_{1actaul}$')
ax.plot(time_vals, theta1_desired, label = r'$\theta_{1desired}$')
ax.plot(time_vals, theta2_vals, label = r'$\theta_{2actual}$')
ax.plot(time_vals, theta2_desired, label = r'$\theta_{2desired}$')
ax.set_ylim(0.32,2.06)
ax.set_xlabel('Time (s)', fontsize = 14)
ax.set_ylabel('Joint Angle (rad)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper right', fontsize = 12.5)



# plt.show()

# fig3,ax = plt.subplots()
# ax.plot(x2_vals, y2_vals)
# ax.plot(x_desired,y_desired)
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('Cartesian Space (m)')
# plt.show()

fig7,ax = plt.subplots()
ax.plot(time_vals, theta1dot_vals)
ax.plot(time_vals, theta1dot_desired)
ax.set_xlabel('Time (s)')
ax.set_ylabel('joint angle velocity 1 (rad/s)')
plt.show()

fig8,ax = plt.subplots()
ax.plot(time_vals, theta2dot_vals)
ax.plot(time_vals, theta2dot_desired)
ax.set_xlabel('Time (s)')
ax.set_ylabel('joint angle velocity 2 (rad/s)')
plt.show()
# theta2 desired and theta2 actual vs time
fig14,ax = plt.subplots()
# ax.plot(time_vals, t1ddot)
ax.plot(time_vals, theta1dotdot_desired)
ax.plot(time_vals, theta2dotdot_desired)
ax.set_xlabel('Time (s)')
ax.set_ylabel('joint angle acceleration 1 (rad/s)')
plt.show()

# fig15,ax = plt.subplots()
# # ax.plot(time_vals, t2ddot)
# ax.set_xlabel('Time (s)')
# ax.set_ylabel('joint angle acceleration 2 (rad/s)')
# plt.show()
# End effector desired trajectory and actual trajectory


# # Controller 1 effort
# fig4,ax = plt.subplots()
# ax.plot(time_vals, tau1arr,label = r'$\tau_{1}$')
# ax.plot(time_vals, tau2arr,label = r'$\tau_{2}$')
# ax.set_ylim(-13.43,36)
# ax.set_xlabel('Time (s)', fontsize = 14)
# ax.set_ylabel('Controller effort (Nm)', fontsize = 14)
# plt.tick_params(axis='both', which='major', labelsize=14)
# plt.legend(loc='upper right', fontsize = 12.5)
# plt.show()


# Error integral for both joints
fig6,ax = plt.subplots()
ax.plot(time_vals, erl1, label = r'$e_{int1}$')
ax.plot(time_vals, erl2, label = r'$e_{int2}$')
ax.set_xlabel('Time (s)', fontsize = 14)
ax.set_ylim(0,0.07)
ax.set_xlim(0,10.23)
ax.set_ylabel('Error Integral (rad)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper right', fontsize = 12.5)
plt.show()

tau34 = []
tau23 = []
energy1 = powerc1l[-1]*dt
energy2 = powerc2l[-1]*dt
for i in range(len(tau1arr)):
    tau34.append(abs(tau1arr[i]))
for i in range(len(tau2arr)):
    tau23.append(abs(tau2arr[i]))
torque1 = max(tau34)
torque2 = max(tau23)
total_effort = tau1l[-1] + tau2l[-1]
total_error = erl1[-1] + erl2[-1]
total_energy = energy1 + energy2
print('Maximum error in Joint 1: ', max(e1))
print('Maximum error in Joint 2: ', max(e2))
print('Total Error in Joint 1:', erl1[-1])
print('Total Error in Joint 2:', erl2[-1])
print('The Total Error in the system is: ', total_error,'rad s')
print("Maximum Operating torque in Joint 1 is: ", torque1, 'Nm')
print("Maximum Operating torque in Joint 2 is: ", torque2, 'Nm')
print('Total Energy Consumption in Joint 1:', energy1,'J')
print('Total Energy Consumption in Joint 2:', energy2,'J')
print('The Total Energy Consumption is: ', total_energy,'J')
print(f'The time elapsed is {time_elapsed:.9f} seconds.')