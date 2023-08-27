import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
L1 = 1
L2 = 1
T = 10
t_span = (0, T)
dt = 0.001
t = np.arange(t_span[0], t_span[1]+dt, dt)
r=0.72
omega = 2 * np.pi / T
# Square Trajectories
# x_desired = np.concatenate((np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)), np.zeros(int(len(t)/4)+1)))
# y_desired = np.concatenate((np.zeros(int(len(t)/4)), np.linspace(0, 1, int(len(t)/4)), np.ones(int(len(t)/4)), np.linspace(1, 0, int(len(t)/4)+1)))

# x_desired = np.concatenate((1.5*np.ones(int(len(t)/5)), np.linspace(1.5, -0.5, int(len(t)/5)), -0.5*np.ones(int(len(t)/5)), np.linspace(-0.5,1.5,int(len(t)/5)),1.5*np.ones(int(len(t)/5)+1)))
# y_desired = np.concatenate((np.linspace(0,1,int(len(t)/5)), np.ones(int(len(t)/5)),np.linspace(1,-1,int(len(t)/5)), -1*np.ones(int(len(t)/5)),np.linspace(-1,0,int(len(t)/5)+1)))

x_desired = np.concatenate((1.7*np.ones(int(len(t)/5)), np.linspace(1.7, 0.7, int(len(t)/5)), 0.7*np.ones(int(len(t)/5)), np.linspace(0.7,1.7,int(len(t)/5)),1.7*np.ones(int(len(t)/5)+1)))
y_desired = np.concatenate((np.linspace(0,0.5,int(len(t)/5)), 0.5*np.ones(int(len(t)/5)),np.linspace(0.5,-0.5,int(len(t)/5)), -0.5*np.ones(int(len(t)/5)),np.linspace(-0.5,0,int(len(t)/5)+1)))
plt.rc('font', family='Helvetica')
# Line Trajectories
# x_desired = np.linspace(-1, 1, int(len(t)))
# y_desired = 1.414*np.ones(int(len(t)))

# Circle Trajectory
# x_desired = 1 + (r * np.cos(omega*t))
# y_desired = r * np.sin(omega*t)

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

tau_array_PID = np.loadtxt('tauvalues_line_PID.txt', delimiter=',')
tau_array_PD = np.loadtxt('tauvalues_line_PD.txt', delimiter=',')
tau_array_P = np.loadtxt('tauvalues_line_P.txt', delimiter=',')
tau_array_PI = np.loadtxt('tauvalues_line_PI.txt', delimiter=',')
tau_array_GS = np.loadtxt('tauvalues_line_GS.txt', delimiter=',')
tau_array_FL = np.loadtxt('tauvalues_line_FL.txt', delimiter=',')
# Access tau1 and tau2 arrays from the loaded 2D array
tau1_values_PID = tau_array_PID[:, 0]
tau2_values_PID = tau_array_PID[:, 1]
tau1_values_PD = tau_array_PD[:, 0]
tau2_values_PD = tau_array_PD[:, 1]
tau1_values_P = tau_array_P[:, 0]
tau2_values_P = tau_array_P[:, 1]
tau1_values_GS = tau_array_GS[:, 0]
tau2_values_GS = tau_array_GS[:, 1]
tau1_values_FL = tau_array_FL[:, 0]
tau2_values_FL = tau_array_FL[:, 1]
tau1_values_PI = tau_array_PI[:, 0]
tau2_values_PI = tau_array_PI[:, 1]


CARTESIAN_array_FL = np.loadtxt('cartesian_values_FL.txt', delimiter=',')
CARTESIAN_array_GS = np.loadtxt('cartesian_values_GS.txt', delimiter=',')
CARTESIAN_array_P = np.loadtxt('cartesian_values_P.txt', delimiter=',')
CARTESIAN_array_PD = np.loadtxt('cartesian_values_PD.txt', delimiter=',')
CARTESIAN_array_PI = np.loadtxt('cartesian_values_PI.txt', delimiter=',')
CARTESIAN_array_PID = np.loadtxt('cartesian_values_PID.txt', delimiter=',')

x1_vals_PID = CARTESIAN_array_PID[:, 0]
x2_vals_PID = CARTESIAN_array_PID[:, 1]
y1_vals_PID = CARTESIAN_array_PID[:, 2]
y2_vals_PID = CARTESIAN_array_PID[:, 3]

x1_vals_PI = CARTESIAN_array_PI[:, 0]
x2_vals_PI = CARTESIAN_array_PI[:, 1]
y1_vals_PI = CARTESIAN_array_PI[:, 2]
y2_vals_PI = CARTESIAN_array_PI[:, 3]
#
x1_vals_PD = CARTESIAN_array_PD[:, 0]
x2_vals_PD = CARTESIAN_array_PD[:, 1]
y1_vals_PD = CARTESIAN_array_PD[:, 2]
y2_vals_PD = CARTESIAN_array_PD[:, 3]
#
x1_vals_P = CARTESIAN_array_P[:, 0]
x2_vals_P = CARTESIAN_array_P[:, 1]
y1_vals_P = CARTESIAN_array_P[:, 2]
y2_vals_P = CARTESIAN_array_P[:, 3]

x1_vals_FL = CARTESIAN_array_FL[:, 0]
x2_vals_FL = CARTESIAN_array_FL[:, 1]
y1_vals_FL = CARTESIAN_array_FL[:, 2]
y2_vals_FL = CARTESIAN_array_FL[:, 3]

x1_vals_GS = CARTESIAN_array_GS[:, 0]
x2_vals_GS = CARTESIAN_array_GS[:, 1]
y1_vals_GS = CARTESIAN_array_GS[:, 2]
y2_vals_GS = CARTESIAN_array_GS[:, 3]






#CONTROL EFFORT Plot
fig,ax = plt.subplots()
ax.plot(t, tau1_values_PID,  label = r'$PID_{j1}$',linewidth = 1.5)
ax.plot(t, tau2_values_PID, label = r'$PID_{j2}$',linewidth = 1.5)
# ax.plot(t, tau1_values_PI,  label = r'$PI_{j1}$',linewidth = 1.5)
# ax.plot(t, tau2_values_PI, label = r'$PI_{j2}$',linewidth = 1.5)
# ax.plot(t, tau1_values_PD,  label = r'$PD_{j1}$',linewidth = 1.5)
# ax.plot(t, tau2_values_PD, label = r'$PD_{j2}$',linewidth = 1.5)
# ax.plot(t, tau1_values_P,  label = r'$P_{j1}$',linewidth = 1.5)
# ax.plot(t, tau2_values_P, label = r'$P_{j2}$',linewidth = 1.5)
ax.plot(t, tau1_values_GS, label =r'$GS_{j1}$',linewidth = 1.5)
ax.plot(t, tau2_values_GS, label = r'$GS_{j2}$',linewidth = 1.5)
ax.plot(t, tau1_values_FL, label = r'$FL_{j1}$',linewidth = 1.5)
ax.plot(t, tau2_values_FL, label = r'$FL_{j2}$',linewidth = 1.5)
plt.tick_params(axis='both', which='major', labelsize=16)
# plt.legend(loc='upper right', fontsize = 12)
legend = plt.legend(loc = 'best', framealpha=0.3, fontsize = 13)
ax.set_xlabel('Time (s)', fontsize = 15)
# ax.set_ylim(0,6.3)
ax.set_xlim(0,10.121)
ax.set_ylabel('Integral Square Error (rad$^2$ s)', fontsize = 15)
plt.show()

fig1,ax = plt.subplots()
# ax.plot(x2_vals_PI, y2_vals_PI,linewidth = 2, label = r'PI', ls = '--',zorder=2)
ax.plot(x2_vals_P, y2_vals_P,linewidth = 2, label = r'P', ls = '--',zorder=2)
# ax.plot(x2_vals_PID, y2_vals_PID,linewidth = 2, label = r'PID', ls = '--')
# ax.plot(x2_vals_GS, y2_vals_GS,linewidth = 2, label = r'PID WITH GS', ls = '--')
ax.plot(x2_vals_FL, y2_vals_FL,linewidth = 2, label = r'FL',ls='--',zorder=2)
ax.plot(x2_vals_PD, y2_vals_PD,linewidth = 2, label = r'PD',ls='--')
ax.plot(x_desired,y_desired,linewidth=3, label = 'Desired Trajectory', zorder=1)
plt.tick_params(axis='both', which='major', labelsize=16)
# plt.legend(loc='upper right', fontsize = 12)
plt.legend(loc = 'best', framealpha=0.3, fontsize = 13)
ax.set_xlabel('x (m)', fontsize = 15)
# ax.set_xlim(-0.2,13)
ax.set_ylabel('y (m)', fontsize = 15)
ax.set_aspect('equal')
# ax.set_xlim(-1.9,3.5)
# ax.set_ylim(-2,1)
plt.show()