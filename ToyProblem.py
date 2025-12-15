import numpy as np 
def HJac(x, PosAngle=False, Omega=False):
    if PosAngle:
        HJac = np.array([[1,0,0,0,0,0,0,0], #rx
                         [0,0,np.cos(x[6]),0,0,-np.sin(x[6]),0,0], #abx
                         [0,0,0,1,0,0,0,0], #ry
                         [0,0,np.sin(x[6]),0,0,np.cos(x[6]),0,0], #aby
                         [0,0,0,0,0,0,1,0], #theta
                         [0,0,0,0,0,0,0,1]]) #omega
    elif Omega:
        HJac = np.array([[0,0,np.cos(x[6]),0,0,-np.sin(x[6]),0,0], #abx
                         [0,0,np.sin(x[6]),0,0,np.cos(x[6]),0,0], #aby
                         [0,0,0,0,0,0,0,1]]) #omega
    else:
        HJac = np.array([[0,0,np.cos(x[6]),0,0,-np.sin(x[6]),0,0], #abx
                         [0,0,np.sin(x[6]),0,0,np.cos(x[6]),0,0]]) #aby
    return HJac
def Hx(x, PosAngle=False, Omega=False):
    if PosAngle:
        Hx = np.array([x[0],x[2]*np.cos(x[6])-x[5]*np.sin(x[6]),\
                       x[3],x[2]*np.sin(x[6])+x[5]*np.cos(x[6]),\
                       x[6],x[7]])
    elif Omega:
        Hx = np.array([x[2]*np.cos(x[6])-x[5]*np.sin(x[6]),\
                       x[2]*np.sin(x[6])+x[5]*np.cos(x[6]),\
                       x[7]])
    else:
        Hx = np.array([x[2]*np.cos(x[6])-x[5]*np.sin(x[6]),\
                       x[2]*np.sin(x[6])+x[5]*np.cos(x[6])])
    return Hx

# %% SIMULATION OPTIONS

#general
endSimOnArrival = True
noDisturbance = False
dt = .1 #s
tf = 150 #s
#angle control
theta_d = 3 #rad (for testing purposes, only used when burns aren't simulated)
kp_theta = 1
ki_theta = 1
kd_theta = 3.5
#burn control
simulateBurns = True

# %% PRE-SIMULATION SETUP
t = np.array([0])
N = int(tf/dt);
#misc config (flags, other vars)
burn1exe = False
burn2exe = False
flag_1 = True
switch_vi = False
deltav_x1 = 0
deltav_y1 = 0
error_theta_sum = 0
control_signal = 0
B = np.array([0.5*dt**2,dt])

# %% ACTUAL VALUE SETUP
origin = [0,0] # %% m
#actual values array setup
r = np.array([np.random.normal(0,10,2)]) #m
v = np.array([np.random.normal(0,0.1,2)]) #m/s
a = np.array([np.random.normal(0,0.01/np.sqrt(dt),2)]) #m/s^2
theta = np.array([np.random.random()*2*np.pi]) #rad
omega = np.array([np.random.normal(0,0.1)]) #rad/s
alpha = 0

# %% SENSOR VALUE SETUP
#sensor uncertainty
sigma_r = 1
sigma_ab = 0.02/np.sqrt(dt)
sigma_theta = 0.1
sigma_omega = 0.01
#position/theta/omega initial sensor values
r_s = np.array([np.random.normal(r[-1],sigma_r,2)]) #m
theta_s = np.random.normal(theta[-1],sigma_theta,1) #rad
omega_s = np.random.normal(omega[-1],sigma_omega,1) #rad/s
#convert acceleration to body frame
a_bx = a[-1][0]*np.cos(theta[0]) - a[-1][1]*np.sin(theta[0])
a_by = a[-1][0]*np.sin(theta[0]) + a[-1][1]*np.cos(theta[0])
#calculate sensor values in body frame (values used for ekf)
ab_s = np.array([np.random.normal([a_bx, a_by],sigma_ab,2)])
#calculate sensor values in inertial frame (values used for plotting)
a_sx = ab_s[-1][0]*np.cos(-theta[0]) - ab_s[-1][1]*np.sin(-theta[0])
a_sy = ab_s[-1][0]*np.sin(-theta[0]) + ab_s[-1][1]*np.cos(-theta[0])
a_s = np.array([[a_sx, a_sy]])

# %% KALMAN FILTER SETUP
from filterpy.kalman import ExtendedKalmanFilter as EKF

#kf for position/velocity/acceleration
sys_ekf = EKF(8,6)
#initial guess
sys_ekf.x = np.array([r_s[-1][0], 0, ab_s[-1][0], r_s[-1][1], 0, ab_s[-1][1],\
                      theta_s[-1], omega_s[-1]])
#state transformation matrix
sys_ekf.F = np.array([[1, dt, 0.5*dt**2, 0, 0, 0, 0, 0], #rx
                     [0, 1, dt, 0, 0, 0, 0, 0], #vx
                     [0, 0, 1, 0, 0, 1, 0, 0], #ax
                     [0, 0, 0, 1, dt, 0.5*dt**2, 0, 0], #ry
                     [0, 0, 0, 0, 1, dt, 0, 0], #vy
                     [0, 0, 0, 0, 0, 1, 0, 0], #ay
                     [0, 0, 0, 0, 0, 0, 1, dt], #theta
                     [0, 0, 0, 0, 0, 0, 0, 1]]) #omega 
#process noise matrix from accleration
sys_ekf.Q = np.array([[dt**4/4, dt**3/2, dt**2/2, 0, 0, 0, 0, 0],
                      [dt**3/2, dt**2, dt, 0, 0, 0, 0, 0],
                      [dt**2/2, dt, 1, 0, 0, 0, 0, 0],
                      [0, 0, 0, dt**4/4, dt**3/2, dt**2/2, 0, 0],
                      [0, 0, 0, dt**3/2, dt**2, dt, 0, 0],
                      [0, 0, 0, dt**2/2, dt, 1, 0, 0],
                      [0, 0, 0, 0, 0, 0, dt**4/4, dt**3/2],
                      [0, 0, 0, 0, 0, 0, dt**3/2, dt**2]])*0.0001
theta_i = [sys_ekf.x[6]]
omega_i = [sys_ekf.x[7]]
r_i = [[sys_ekf.x[0],sys_ekf.x[3]]]
v_i = [[sys_ekf.x[1],sys_ekf.x[4]]]
a_i = [[sys_ekf.x[2],sys_ekf.x[5]]]

# %% SIMULATION START
for i in range(1,N+1): 
    
    # %% CALCULATE ACTUAL VALUES
    
    if noDisturbance:
        a[0] = np.array([0,0])
        a = np.append(a, np.array([[0,0]]), axis=0)
    else:
        a = np.append(a, np.array([np.random.normal(0,0.01/np.sqrt(dt),2)]), axis=0)
        
    #add time
    t = np.append(t,np.around(np.array([t[-1]+dt]),decimals=6))
    
    #calculate velocity and position
    v_x = v[i-1][0] + a[i][0] * dt
    v_y = v[i-1][1] + a[i][1] * dt
    v = np.append(v, np.array([[v_x, v_y]]), axis=0)
    
    r_x = r[i-1][0] + v[i][0] * dt + 0.5*a[i][0] * dt**2
    r_y = r[i-1][1] + v[i][1] * dt + 0.5*a[i][1] * dt**2
    r = np.append(r, np.array([[r_x, r_y]]), axis=0)
    
    #calculate theta/omega
    omega = np.append(omega,np.array([omega[-1]+alpha*dt]), axis=0)
    theta = np.append(theta,np.array([theta[-1] + omega[-1]*dt + 0.5*alpha*dt**2]), axis=0)
    
    
    # %% CALCULATE SENSOR VALUES
    
    #position/angle every 1 s
    if t[i]*10 % 10 == 0:
        r_s = np.append(r_s,np.array([np.random.normal(r[i],sigma_r,2)]),axis=0)
        theta_s = np.append(theta_s,np.random.normal(theta[i],sigma_theta,1),axis=0)
    #angle rate every 0.1 s
    if t[i]*10 % 1 == 0:
        omega_s = np.append(omega_s,np.random.normal(omega[i],sigma_omega,1),axis=0)
    #convert acceleration to body frame
    a_bx = a[-1][0]*np.cos(theta[0]) - a[-1][1]*np.sin(theta[0])
    a_by = a[-1][0]*np.sin(theta[0]) + a[-1][1]*np.cos(theta[0])
    #calculate sensor values in body frame
    ab_s = np.append(ab_s, [np.random.normal([a_bx, a_by],sigma_ab,2)], axis=0)
    #convert sensor values back to inertial frame
    a_sx = ab_s[-1][0]*np.cos(-theta[0]) - ab_s[-1][1]*np.sin(-theta[0])
    a_sy = ab_s[-1][0]*np.sin(-theta[0]) + ab_s[-1][1]*np.cos(-theta[0])
    a_s = np.append(a_s, np.array([[a_sx, a_sy]]), axis=0)
    
    
    # %% CALCULATE INTERPOLATED SENSOR VALUES
    
    #pos/acc/angle/omega (update every 1s)
    if t[i] * 10 % 10 == 0:
        #measurement uncertainty matrix
        sys_ekf.R = np.array([[sigma_r**2, 0, 0, 0, 0, 0],
                             [0, sigma_ab**2, 0, 0, 0, 0],
                             [0, 0, sigma_r**2, 0, 0, 0],
                             [0, 0, 0, sigma_ab**2, 0, 0],
                             [0, 0, 0, 0, sigma_theta**2, 0],
                             [0, 0, 0, 0, 0, sigma_omega**2]])
        #measurements matrix
        z = np.array([r_s[-1][0], ab_s[-1][0], r_s[-1][1], ab_s[-1][1],\
                      theta_s[-1],omega_s[-1]])
        sys_ekf.predict_update(z,HJac,Hx,(True,True),(True,True))
    
    #acc/omega (update every 0.1s)
    elif t[i] * 10 % 1 == 0:
        #measurement uncertainty matrix
        sys_ekf.R = np.array([[sigma_ab**2, 0, 0],
                             [0, sigma_ab**2, 0],
                             [0, 0, sigma_omega**2]])
        #measurements matrix
        z = np.array([ab_s[-1][0], ab_s[-1][1], omega_s[-1]])
        sys_ekf.predict_update(z,HJac,Hx,(False,True),(False,True))
        
    #acc/omega (update every time step)
    else:
        #measurement uncertainty matrix
        sys_ekf.R = np.array([[sigma_ab**2, 0, 0],
                             [0, sigma_ab**2, 0]])
        #measurements matrix
        z = np.array([ab_s[-1][0], ab_s[-1][1]])
        sys_ekf.predict_update(z,HJac,Hx,(False,False),(False,False))
        
    r_i_new = [[sys_ekf.x[0], sys_ekf.x[3]]]
    v_i_new = [[sys_ekf.x[1], sys_ekf.x[4]]]
    a_i_new = [[sys_ekf.x[2], sys_ekf.x[5]]]
    theta_i_new = [sys_ekf.x[6]]
    omega_i_new = [sys_ekf.x[7]]
    r_i = np.append(r_i, r_i_new, axis=0)
    v_i = np.append(v_i, v_i_new, axis=0)
    a_i = np.append(a_i, a_i_new, axis=0)
    theta_i = np.append(theta_i, theta_i_new, axis=0)
    omega_i = np.append(omega_i, omega_i_new, axis=0)
    
    
    # %% CONTROL ANGLE
    
    #control to angle of origin for first burn
    if simulateBurns and not burn1exe:
        angle_to_origin = (np.arctan2(origin[1] - r_i[-1][1], origin[0] - r_i[-1][0]) + 2*np.pi) % (2*np.pi)
        theta_d = angle_to_origin
    #after first burn, change desired angle by 180 deg to cancel out velocity
    if simulateBurns and burn1exe and flag_1:
        theta_d = theta_d - np.pi
        flag_1 = False
    #angle error
    error_theta = theta_d - theta_i[-1]
                
    #pid components
    error_theta_p = kp_theta * error_theta
    if abs(error_theta) < 1:
        error_theta_sum += error_theta * dt  #cumulative error
        error_theta_i = ki_theta * error_theta_sum
    else:
        error_theta_i = 0
    error_theta_d = kd_theta * -omega_i[-1]
    
    #update angular acceleration based on pid
    alpha = error_theta_p + error_theta_i + error_theta_d
    
    
    # %% CALCULATE IMPULSIVE THRUSTS
    
    if simulateBurns:
        #cond true when within 0.01 rad of origin
        angle_cond = abs(theta_i[-1]-theta_d) < .01
        #execute first burn if within 0.01 rad of origin and after 40 seconds
        
        if not burn1exe and angle_cond and int(t[i])>40:
            #find delta v needed (to_to_origin is 20 because avg settling time)
            d_to_origin = np.sqrt((origin[1] - r_i[-1][1])**2 + (origin[0] - r_i[-1][0])**2)
            t_to_origin = 20
            t_since_burn1 = 0
            deltav_req = d_to_origin/t_to_origin
            deltav_x1 = deltav_req*np.cos(angle_to_origin) - v_i[-1][0] 
            deltav_y1 = deltav_req*np.sin(angle_to_origin) - v_i[-1][1]
            
            #loc of first burn (used only for plotting hence actual values)
            loc_burn1x = r[-1][0]
            loc_burn1y = r[-1][1]
            
            #update actual velocities
            v[-1][0] = v[-1][0] + deltav_x1
            v[-1][1] = v[-1][1] + deltav_y1
            
            #update velocities in EKF
            sys_ekf.x[1] = deltav_x1+sys_ekf.x[1]
            sys_ekf.x[4] = deltav_y1+sys_ekf.x[4]
            
            burn1exe = True
            print(f'Burn 1 done at {t[i]}s. First location: {r[i]}, with new V: {v[i]}')
        
        #consider second burn after first burn executed
        if burn1exe:
            #calculate time since last burn, cond true after 40 s since burn1
            t_since_burn1 += dt
            time_cond = t_since_burn1 > t_to_origin
            
            #execute second burn after 40 seconds from initial burn (burn2exe again)
            if time_cond and not burn2exe:
                #delta v is set to cancel out current velocity
                deltav_x2 = -v_i[-1][0] 
                deltav_y2 = -v_i[-1][1]
                v[i][0] = v[i][0] + deltav_x2
                v[i][1] = v[i][1] + deltav_y2
                
                #loc of first burn (used only for plotting hence actual values)
                loc_burn2x = r[-1][0]
                loc_burn2y = r[-1][1]
                
                #update velocities in EKF
                sys_ekf.x[1] = deltav_x2+v_i[-1][0]
                sys_ekf.x[4] = deltav_y2+v_i[-1][1]
                
                burn2exe = True
                print(f'Burn 2 done at {t[i]}s. Final location: {r[i]}')
                if endSimOnArrival:
                    break     
               
# %% PLOTS
import matplotlib.pyplot as plt

plotPos = True
plotPosS = True
plotPosI = True
if plotPos:
    plt.figure(figsize=(10, 6))  # %% Set the figure size
    plt.plot(r[:, 0], r[:, 1], label='Actual Position', color='blue', linewidth=2)  # %% Actual position
    if plotPosS:
        plt.scatter(r_s[:, 0], r_s[:, 1], s=10, c='red', label='Sensor Position')  # %% Sensor positions
    if plotPosI:
        plt.scatter(r_i[:, 0], r_i[:, 1], s=5, c='green', label='Interpolated Position')  # %% Interpolated positions
    if simulateBurns:
        plt.arrow(loc_burn1x,loc_burn1y,deltav_x1,deltav_y1,head_width=.5,head_length=1,ec='black')
    plt.title('Position of the Object Over Time')  # %% Title
    plt.xlabel('X Position (m)')  # %% X-axis label
    plt.ylabel('Y Position (m)')  # %% Y-axis label
    plt.legend()  # %% Show legend
    plt.grid(True)  # %% Add grid lines
    plt.axis('equal')  # %% Set equal scaling for both axes
    plt.show()  # %% Display the plot
    
plotVel = True
plotVelI = True
if plotVel:
    plt.figure(figsize=(10, 6))  # %% Set the figure size
    plt.plot(t, v[:,0], label='Velocity_x (v)', color='blue', linewidth=2)  # %% Velocity over time
    if plotVelI:
        plt.scatter(np.linspace(0, t[-1], len(v_i[:,0])), v_i[:,0], s=10, c='green', label='Interpolated Velocity_x')  # %% Interpolated velocities
    plt.title('Velocity_x of the Object Over Time')  # %% Title
    plt.xlabel('Time (s)')  # %% X-axis label
    plt.ylabel('Velocity_x (m/s)')  # %% Y-axis label
    plt.legend()  # %% Show legend
    plt.grid(True)  # %% Add grid lines
    plt.show()  # %% Display the plot
    
plotAcc = False
plotAccS = False
plotAccI = False
if plotAcc:
    plt.figure(figsize=(10, 6))  # %% Set the figure size
    plt.plot(t, a[:,0], label='Acceleration_x (a)', color='blue', linewidth=1)  # %% Acceleration over time
    if plotAccS:
        plt.scatter(np.linspace(0, t[-1], len(a_s[:,0])), a_s[:,0], s=10, c='red', label='Sensor Accelerations_x')  # %% Sensor accelerations
    if plotAccI:
        plt.scatter(np.linspace(0, t[-1], len(a_i[:,0])), a_i[:,0], s=3, c='green', label='Interpolated Accelerations_x')  # %% Interpolated accelerations
    plt.title('Acceleration_x of the Object Over Time')  # %% Title
    plt.xlabel('Time (s)')  # %% X-axis label
    plt.ylabel('Acceleration_x (m/s^2)')  # %% Y-axis label
    plt.legend()  # %% Show legend
    plt.grid(True)  # %% Add grid lines
    plt.show()  # %% Display the plot
plotAng = True
plotAngS = True
plotAngI = True
if plotAng:
    plt.figure(figsize=(10, 6))  # %% Set the figure size
    plt.plot(t, theta, label='Theta (Angle)', color='blue', linewidth=1)  # %% Theta over time
    if plotAngS:
        plt.scatter(np.linspace(0, t[-1], len(theta_s)), theta_s, s=5, c='red', label='Sensor Angles')  # %% Sensor angles
    if plotAngI:
        plt.scatter(np.linspace(0, t[-1], len(theta_i)), theta_i, s=3, c='green', label='Interpolated Angles')  # %% Interpolated angles
    plt.title('Angle of the Object Over Time')  # %% Title
    plt.xlabel('Time (s)')  # %% X-axis label
    plt.ylabel('Angle (rad)')  # %% Y-axis label
    plt.legend()  # %% Show legend
    plt.grid(True)  # %% Add grid lines
    plt.show()  # %% Display the plot
    
plotOmega = False
plotOmegaS = True
plotOmegaI = True
if plotOmega:
    plt.figure(figsize=(10, 6))  # %% Set the figure size
    plt.plot(t, omega, label='Angular Velocity (ω)', color='blue', linewidth=2)  # %% Angular velocity over time
    if plotOmegaS:
        plt.scatter(np.linspace(0, t[-1], len(omega_s)), omega_s, s=3, c='red', label='Sensor Angular Velocities')  # %% Sensor angular velocities
    if plotOmegaI:
        plt.scatter(np.linspace(0, t[-1], len(omega_i)), omega_i, s=5, c='green', label='Interpolated Angular Velocities')  # %% Interpolated angular velocities
    plt.title('Angular Velocity of the Object Over Time')  # %% Title
    plt.xlabel('Time (s)')  # %% X-axis label
    plt.ylabel('Angular Velocity (rad/s)')  # %% Y-axis label
    plt.legend()  # %% Show legend
    plt.grid(True)  # %% Add grid lines
    plt.show()  # %% Display the plot

# %% STDEV OF COMPONENTS

print(f'STDEV -- POSITION: {np.std(r_i-r):.5f}')
print(f'STDEV -- VELOCITY: {np.std(v_i-v):.5f}')
print(f'STDEV -- ACCELERATION: {np.std(a_i-a):.5f}')
print(f'STDEV -- THETA: {np.std(theta_i-theta):.5f}')
if not simulateBurns:
    print(f'\tSTDEV -- THETA @ SS: {np.std(theta[int(20/dt):]-theta_d):.5f}')
print(f'STDEV -- OMEGA: {np.std(omega_i-omega):.5f}')
