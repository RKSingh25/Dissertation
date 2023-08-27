import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from matplotlib import rcParams
import time

# Set the default font to Helvetica
plt.rc('font', family='Helvetica')
# Define dynamic parameters of the robot
m1 = 1  # mass of link 1
m2 = 1  # mass of link 2
L1 = 1  # length of link 1
L2 = 1  # length of link 2
g = 9.81  # gravitational acceleration
Kp1 = 120
Kp2 = 120
Kd1 = 30
Kd2 = 30

# Define simulation time and time step
T = 10
t_span = (0, T)
dt = 0.001
t = np.arange(t_span[0], t_span[1]+dt, dt)
r = input("circle radius? ") #0.72
x_offset = input("distance from centre in x? ") #1
y_offset = input("distance from centre in y? ") #0
# Define desired trajectory for end-effector position in Cartesian space

# Square Trajectories
# x_desired = np.concatenate((np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)), np.zeros(int(len(t)/4)+1)))
# y_desired = np.concatenate((np.zeros(int(len(t)/4)), np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)+1)))

# x_desired = np.concatenate((1.5*np.ones(int(len(t)/5)), np.linspace(1.5, -0.5, int(len(t)/5)), -0.5*np.ones(int(len(t)/5)), np.linspace(-0.5,1.5,int(len(t)/5)),1.5*np.ones(int(len(t)/5)+1)))
# y_desired = np.concatenate((np.linspace(0,1,int(len(t)/5)), np.ones(int(len(t)/5)),np.linspace(1,-1,int(len(t)/5)), -1*np.ones(int(len(t)/5)),np.linspace(-1,0,int(len(t)/5)+1)))

# x_desired = np.concatenate((xx2*np.ones(int(len(t)/5)), np.linspace(xx2, xx1, int(len(t)/5)), xx1*np.ones(int(len(t)/5)), np.linspace(xx1,xx2,int(len(t)/5)),xx2*np.ones(int(len(t)/5)+1)))
# y_desired = np.concatenate((np.linspace(yy1,yy2,int(len(t)/5)), yy2*np.ones(int(len(t)/5)),np.linspace(yy2,yy3,int(len(t)/5)), yy3*np.ones(int(len(t)/5)),np.linspace(yy3,yy1,int(len(t)/5)+1)))

# Line Trajectories
# x_desired = np.linspace(-1, 1, int(len(t)))
# y_desired = 1.414*np.ones(int(len(t)))

# Circle Trajectory
# x_desired = 1 + (r * np.cos(omega*t))
# y_desired = r * np.sin(omega*t)
omega = 2 * np.pi / T
x_desired = x_offset + (r * np.cos(omega*t))
y_desired = y_offset + (r * np.sin(omega*t))

#Unit Step

# theta1_desired = np.concatenate((-np.pi/2*np.ones(int(len(t)/2)), 0.5*np.ones(int(len(t)/2)+1)))
# theta2_desired = np.concatenate((np.pi*np.ones(int(len(t)/2)), np.pi*np.ones(int(len(t)/2)+1)))
# theta1_desired = np.concatenate((np.ones(int(len(t)/4)), 2*np.ones(int(len(t)/4)), -2*np.ones(int(len(t)/4)), -1*np.ones(int(len(t)/4)+1)))
# theta2_desired = np.concatenate((np.ones(int(len(t)/4)), 0*np.ones(int(len(t)/4)), 0.5*np.ones(int(len(t)/4)), -1.5*np.ones(int(len(t)/4)+1)))
# theta2_desired = np.concatenate((np.linspace(0,np.pi/4,int(len(t)/4)),np.linspace(np.pi/4,np.pi/8,int(len(t)/4)),np.linspace(np.pi/8,np.pi/6,int(len(t)/8)),np.linspace(np.pi/6,np.pi/3,int(3*(len(t)/8)+1))))
# theta1_desired = np.concatenate((np.linspace(-np.pi/2,0,int(len(t)/4)),np.linspace(0,-np.pi/4,int(len(t)/8)),np.linspace(-np.pi/4,1,int(len(t)/4)),np.linspace(1,-np.pi/8,int(3*(len(t)/8)+1))))
# Compute corresponding desired joint angles using inverse kinematics
theta1_desired = np.zeros_like(x_desired)
theta2_desired = np.zeros_like(x_desired)
x_list = []
y_list = []
the1 = []
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

for i in range(len(x_desired)):
    x = x_desired[i]
    y = y_desired[i]
    x_list.append(x)
    y_list.append(y)
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
    the = theta_desired(theta1_desired[i], theta2_desired[i])
    the1.append(the)
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
theta_dot_desired = np.column_stack((theta1dot_desired, theta2dot_desired)).T

theta1dotdot_desired = np.diff(theta1dot_desired) / dt
theta1dotdot_desired = np.append(theta1dotdot_desired, theta1dotdot_desired[-1])
theta2dotdot_desired = np.diff(theta2dot_desired) / dt
theta2dotdot_desired = np.append(theta2dotdot_desired, theta2dotdot_desired[-1])
theta_double_dot_desired = np.column_stack((theta1dotdot_desired, theta2dotdot_desired)).T

#Define inital Conditions
theta1 = theta1_desired[0]
theta2 = theta2_desired[0]
theta1_dot = theta1dot_desired[0]
theta2_dot = theta2dot_desired[0]
#
# Define control gains
k_p = np.array([[Kp1, 0], [0, Kp2]])
k_d = np.array([[Kd1, 0], [0, Kd2]])
J1 = (1/12)*m1*L1**2
J2 = (1/12)*m2*L2**2

# Start plot


def Mass_Matrix(theta2):
    global J1, J2
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
theta1_vals = []
theta2_vals = []
theta1d_vals = []
theta2d_vals = []
theta1d_dot_vals = []
theta2d_dot_vals = []
error1_integral = 0
error2_integral = 0
x1_vals = []
x2_vals = []
y1_vals = []
y2_vals = []
time_vals = []
tau1=[]
tau2=[]
e1 = 0
e1l = []
e2 = 0
e2l = []
errror1 = []
errror2 = []
cumulative_error1 = 0
erl1 = []
cumulative_error2 = 0
erl2 = []
tau1l = []
tau2l = []
tau1c = 0
tau2c = 0
powerc1l = []
powerc2l = []
powerc1 = 0
powerc2 = 0
error_int1 = 0
error_int2 = 0
max_torque1 = 1500
max_torque2 = 1500
wind_torque = np.concatenate((1.5*np.zeros(int(2*(len(t)/3))), np.linspace(0, 14.38, int(len(t)/3)+1)))

def feedback_linearisation_function(theta,thetadot,theta_desired,thetadot_desired,k_p,k_d):

    # Compute control input
    M = Mass_Matrix(theta[1][0])
    C = Coriolis_Matrix(thetadot[0][0],thetadot[1][0],theta[1][0])
    G = Gravity_Vector(theta[0][0],theta[1][0])
    error = theta_desired - theta
    error_dot = thetadot_desired - thetadot
    tau_controller = k_p@error + k_d@error_dot
    tau_linearised = M @ (tau_controller) + C @ thetadot + G

    return tau_linearised
start_time = time.time()
for i in range(len(t)):

    theta1_vals.append(theta1)
    theta2_vals.append(theta2)
    theta = np.array([[theta1], [theta2]])
    thetadot = np.array([[theta1_dot], [theta2_dot]])
    theta_desired = list(the1[i])
    theta_desired = np.array(theta_desired).reshape(-1, 1)
    thetadot_desired = [[theta_dot_desired[0][i]],[theta_dot_desired[1][i]]]
    thetadoubledot_desired = [[theta_double_dot_desired[0][i]],[theta_double_dot_desired[1][i]]]
    error_theta1 = theta1_desired[i] - theta1
    error_theta2 = theta2_desired[i] - theta2
    error_int1 += error_theta1*dt
    error_int2 += error_theta2*dt
    errror1.append(abs(error_theta1))
    errror2.append(abs(error_theta2))
    cumulative_error1 += (abs(error_theta1)**2)*dt
    erl1.append(cumulative_error1)
    cumulative_error2 += (abs(error_theta2)**2)*dt
    erl2.append(cumulative_error2)
    disturbance = np.zeros((2, 1))
    # disturbance[0] = wind_torque[i]  # add disturbance torque to first joint
    # disturbance[1] = wind_torque[i]  # add disturbance torque to second joint
    invM = np.linalg.inv(Mass_Matrix(theta2))
    C = Coriolis_Matrix(theta1_dot, theta2_dot, theta2)
    G = Gravity_Vector(theta1, theta2)
    tau_vector = feedback_linearisation_function(theta,thetadot,theta_desired,thetadot_desired,k_p,k_d)
    tau_vector[0][0] = np.clip(tau_vector[0][0], -max_torque1, max_torque1)
    tau_vector[1][0] = np.clip(tau_vector[1][0], -max_torque2, max_torque2)
    tau_vector_disturbed = tau_vector
    thetad_dot_vector = invM @ (tau_vector_disturbed - C@theta_dot_vector - G)
    # tau1.append(abs(tau_vector[0][0]))
    # tau2.append(abs(tau_vector[1][0]))
    tau1.append(cumulative_error1)
    tau2.append(cumulative_error2)
    tau1c += abs(tau_vector[0][0])*dt
    tau2c += abs(tau_vector[1][0])*dt
    powerc1 += abs(tau_vector[0][0] * theta1_dot)
    powerc2 += abs(tau_vector[1][0] * theta2_dot)
    tau1l.append(tau1c)
    tau2l.append(tau2c)
    theta1_dot += thetad_dot_vector[0][0]*dt
    theta2_dot += thetad_dot_vector[1][0]*dt
    theta1 += theta1_dot*dt
    theta2 += theta2_dot*dt
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
total_time = end_time - start_time
trajectory = []

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

fig,ax = plt.subplots()
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


# create your animation object and start it
frames = range(0, len(t), 11)
ani = animation.FuncAnimation(fig, update, frames=frames, fargs=(line,trajectory_line,), interval=1000*dt, blit=True, repeat=False)
plt.show()

tau1_array = np.array(tau1)
tau2_array = np.array(tau2)
np.savetxt('tauvalues_line_FL.txt', np.column_stack((tau1, tau2)), delimiter=',')
x1_array = np.array(x1_vals)
x2_array = np.array(x2_vals)
y1_array = np.array(y1_vals)
y2_array = np.array(y2_vals)
np.savetxt('cartesian_values_FL.txt', np.column_stack((x1_array, x2_array,y1_array,y2_array)), delimiter=',')
fig3,ax = plt.subplots()
ax.plot(x_desired, y_desired)
ax.plot(x2_vals, y2_vals)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Cartesian space (m)')
plt.show()
#
fig3,ax = plt.subplots()
ax.plot(time_vals,theta1_vals)
ax.plot(time_vals,theta1_desired, color='orange')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Joint angle 1 (rad)')
plt.show()

fig4,ax = plt.subplots()
ax.plot(time_vals,theta2_vals)
ax.plot(time_vals,theta2_desired, color='orange')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Joint angle 2 (rad)')
plt.show()

fig5,ax = plt.subplots()
ax.plot(time_vals,tau1)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Control Effort 1')
plt.show()

fig6,ax = plt.subplots()
ax.plot(time_vals,tau2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Control Effort 2')
plt.show()

fig7,ax = plt.subplots()
ax.plot(time_vals, erl1)
ax.plot(time_vals, erl2)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Error Integral for both joints')
plt.show()
total_error = erl1[-1] + erl2[-1]
total_effort = tau1l[-1] + tau2l[-1]
energy1 = powerc1*dt
energy2 = powerc2*dt
print('Maximum error in Joint 1:', max(errror1))
print('Maximum error in Joint 2:', max(errror2))
print('Error in Joint 1:', cumulative_error1)
print('Error in Joint 2:', cumulative_error2)
print('The Total Error in the system is: ', total_error)
print('Control Effort in Joint 1:', tau1l[-1])
print('Control Effort in Joint 2:', tau2l[-1])
print('The maximum operating torque in joint 1 is:', max(tau1))
print('The maximum operating torque in joint 2 is:', max(tau2))
print('The Total Control Effort in the system is: ', total_effort,'Nm')
print(f'The time elapsed is {total_time:.9f} seconds.')
print(energy1 + energy2)

# theta1 desired and theta1 actual vs time
fig,ax = plt.subplots()
ax.plot(time_vals, theta1_vals, label = r'$\theta_{1actaul}$')
ax.plot(time_vals, theta1_desired, label = r'$\theta_{1desired}$')
ax.plot(time_vals, theta2_vals, label = r'$\theta_{2actual}$')
ax.plot(time_vals, theta2_desired, label = r'$\theta_{2desired}$')
# ax.set_ylim(0.3,2.43)
ax.set_xlabel('Time (s)', fontsize = 14)
ax.set_ylabel('Joint Angle (rad)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper right', fontsize = 12.5)

# # Controller 1 effort
fig4,ax = plt.subplots()
ax.plot(time_vals, tau1,label = r'$\tau_{1}$')
ax.plot(time_vals, tau2,label = r'$\tau_{2}$')
# ax.set_ylim(-0.7,17.43)
ax.set_xlabel('Time (s)', fontsize = 14)
ax.set_ylabel('Controller effort (Nm)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper right', fontsize = 12.5)
plt.show()


# Error integral for both joints
fig6,ax = plt.subplots()
ax.plot(time_vals, erl1, label = r'$e_{int1}$')
ax.plot(time_vals, erl2, label = r'$e_{int2}$')
ax.set_xlabel('Time (s)', fontsize = 14)
# ax.set_ylim(0,0.0046)
# ax.set_xlim(0,10.23)
ax.set_ylabel('Error Integral (rad)', fontsize = 14)
plt.tick_params(axis='both', which='major', labelsize=14)
plt.legend(loc='upper right', fontsize = 12.5)
plt.show()