clear all
close all
%% Constants

r_t=[2.8848e+06;6.2227e+06;0].*10^-3; % current postion of the target cubesat currently just a guess for now along the orbit selected will use data from ADCS in km
v_t=[919.0035;-426.0376;7.5941e+03].*10^-3; % current velocity of the target cubesat in km/s
mu=398600;
r_t_s=[normrnd(r_t(1),1.2*10^-3),normrnd(r_t(2),1.2*10^-3),normrnd(r_t(3),2.1*10^-3)]; % sensor value of the current postion of the target spacecraft
v_t_s=[normrnd(v_t(1),.09*10^-3),normrnd(v_t(2),.09*10^-3),normrnd(v_t(3),.09*10^-3)]; % sensor value of the current velocity of the target spacecraft
[H_t,i_t,e_t,big_omega_t,small_omega_t,theta_t]=COE(r_t,v_t,mu); % classical orbital elements of the target cubesat
[H_t_s,i_t_s,e_t_s,big_omega_t_s,small_omega_t_s,theta_t_s]=COE(r_t_s,v_t_s,mu); % classical orbital elements of the target cubesat

r_c=[2.8848e+06;6.2229e+06;0].*10^-3; %current postion of the chaser cubesat
v_c=[918.9902;-426.0314;7.5940e+03].*10^-3; % current velocity of the chaser cubesat
r_c_s=[normrnd(r_c(1),1.2*10^-3),normrnd(r_c(2),1.2*10^-3),normrnd(r_c(3),2.1*10^-3)]; % sensor value of the current postion of the chaser spacecraft
v_c_s=[normrnd(v_c(1),.09*10^-3),normrnd(v_c(2),.09*10^-3),normrnd(v_c(3),.09*10^-3)]; % sensor value of the current velocity of the chaser spacecraft
[H_c,i_c,e_c,big_omega_c,small_omega_c,theta_c]=COE(r_c,v_c,mu); % classical orbital elements of the chaser cubesat
[H_c_s,i_c_s,e_c_s,big_omega_c_s,small_omega_c_s,theta_c_s]=COE(r_c_s,v_c_s,mu); % classical orbital elements of the chaser cubesat
%%
%r_t=550+6378; % radius in km of orbit
R_t=norm(r_t);
n_t=sqrt(mu/(R_t^3));
R_c=norm(r_c);
C_t=COE_To_ROE(big_omega_t,small_omega_t,theta_t,i_t); % creates a matrix that is used to switch frames
C_c=COE_To_ROE(big_omega_c,small_omega_c,theta_c,i_c); % creates a matrix that is used to switch frames

R_t_s=norm(r_t_s);
n_t_s=sqrt(mu/(R_t_s^3));
R_c_s=norm(r_c_s);
C_t_s=COE_To_ROE(big_omega_t_s,small_omega_t_s,theta_t_s,i_t_s); % creates a matrix that is used to switch frames
C_c_s=COE_To_ROE(big_omega_c_s,small_omega_c_s,theta_c_s,i_c_s); % creates a matrix that is used to switch frames

%% Initializing the orbits
r_T=[norm(r_t),0,0]; % The position vector from the center of the earth to the target
r_C=[norm(r_c),0,0]; % The position vector from the center of the earth to the chaser
v_T=[0,sqrt(mu/R_t),0]; % Assuming that the tracker is in a circular orbit this is the velocity vector since circular velocity is only in tangential direction
v_C=[0,sqrt(mu/R_c),0]; % Same assumtion for the chaser
delta_v0=(inv(C_t)*C_c*v_C.'-v_T.').';
delta_r0=(inv(C_t)*C_c*r_C.'-r_T.').';

r_T_s=[norm(r_t_s),0,0]; % The position vector from the center of the earth to the target from sensor value
r_C_s=[norm(r_c_s),0,0]; % The position vector from the center of the earth to the chaser from sensor value
v_T_s=[0,sqrt(mu/R_t_s),0]; % Assuming that the tracker is in a circular orbit this is the velocity vector since circular velocity is only in tangential direction for sensor
v_C_s=[0,sqrt(mu/R_c_s),0]; % Same assumtion for the chaser from sensor
delta_v0_s=(inv(C_t_s)*C_c_s*v_C_s.'-v_T_s.').';
delta_r0_s=(inv(C_t_s)*C_c_s*r_C_s.'-r_T_s.').';
delta_r_Observation=[0,.010,0];

tf=3600+45*60;
t=linspace(0,tf,tf);
position=zeros(tf,3);
velocity=zeros(tf,3);
b=.250;
c=.400;
a=sqrt(b^2+c^2);
theta_first=acos(c/a);
T_first=a*cos(theta_first)-c;
theta_t=linspace(theta_first,theta_first+2*pi,tf);
%theta=linspace(0,2*pi);
R=.250*sin(theta_t);
T=a*cos(theta_t)-c;
N=-.080*cos(theta_t);
ideal_standby=[R.',T.',N.'];
for i=1:tf
    position(i,:)=CW_Matrix_pos(t(i),delta_r0,delta_v0,n_t);
    velocity(i,:)=CW_Matrix_velo(t(i),delta_r0,delta_v0,n_t);
end

t_t=15*60; % transfer time to observation
t_t_vector=linspace(0,t_t,t_t);
delta_v=zeros(1,tf);
delta_v_t=zeros(tf,3);
delta_v_t_standby=zeros(tf,3);
delta_v_standby=zeros(1,tf);
position_standby=zeros(tf,3);
velocity_standby=zeros(tf,3);

delta_vt0_standby=transfer_to_observation(5,ideal_standby(1,:),position(1,:),n_t);
delta_v_t_standby(1,:)=(CW_Matrix_velo(5,position(1,:),delta_vt0_standby,n_t)).';
delta_v_standby(1)=norm(-delta_v_t_standby)+norm(delta_vt0_standby-velocity(1,:));
position_standby(1,:)=CW_Matrix_pos(0,position(1,:),delta_v_t_standby(1,:)-velocity(1,:),n_t);
velocity_standby(1,:)=CW_Matrix_velo(0,position(1,:),delta_v_t_standby(1,:)-velocity(1,:),n_t);
for i=2:tf
    position_standby(i,:)=CW_Matrix_pos(t(i),position_standby(1,:),delta_v_t_standby(1,:)-velocity(1,:),n_t);
    velocity_standby(i,:)=CW_Matrix_velo(t(i),position_standby(1,:),delta_v_t_standby(1,:)-velocity(1,:),n_t);
    if i>=5
        position_standby(i,:)=CW_Matrix_pos(t(i),position_standby(1,:),delta_v_t_standby(1,:),n_t);
        velocity_standby(i,:)=CW_Matrix_velo(t(i),position_standby(1,:),delta_v_t_standby(1,:),n_t);
    end
end
% for i=2:2:tf-1
%     delta_vt0_standby=transfer_to_observation(.5,ideal_standby(i,:),position_standby(i-1,:),n_t);
%     delta_v_t_standby(i,:)=(CW_Matrix_velo(.5,position_standby(i-1,:),delta_vt0_standby,n_t)).';
%     delta_v_standby(i)=norm(-delta_v_t)+norm(delta_vt0_standby-velocity_standby(i-1,:));
%     position_standby(i,:)=CW_Matrix_pos(1,position_standby(i-1,:),delta_v_t_standby(i,:)-velocity_standby(i-1,:),n_t);
%     velocity_standby(i,:)=CW_Matrix_velo(1,position_standby(i-1,:),delta_v_t_standby(i,:)-velocity_standby(i-1,:),n_t);
%     position_standby(i+1,:)=CW_Matrix_pos(1,position_standby(i,:),delta_v_t_standby(i,:),n_t);
%     velocity_standby(i+1,:)=CW_Matrix_velo(1,position_standby(i,:),delta_v_t_standby(i,:),n_t);
% end

% figure;
% subplot(2, 1, 1);
% plot(t,velocity_standby(:,1))
% hold on
% plot(t,velocity_standby(:,2))
% plot(t,velocity_standby(:,3))
% xlabel("Time (sec)")
% ylabel("velocity (km/s)")
% legend("Radial","Tangential","Normal")
% title('Relative Velocity of the Chaser')
% hold off
% subplot(2, 1, 2);
% plot(t,position_standby(:,1))
% hold on
% plot(t,position_standby(:,2))
% plot(t,position_standby(:,3))
% xlabel("Time (sec)")
% ylabel("distance (km)")
% legend("Radial","Tangential","Normal")
% title('Relative Position of the Chaser')
% hold off


for i=1:tf
    delta_vt0_t=transfer_to_observation(t_t,delta_r_Observation,position(i,:),n_t);
    delta_v_t(i,:)=(CW_Matrix_velo(t_t,position(i,:),delta_vt0_t,n_t)).';
    delta_v(i)=norm(-delta_v_t)+norm(delta_vt0_t-velocity(i,:));
end
[min_delta_v_cost,time]=min(delta_v);
position_o=zeros(t_t,3);
velocity_o=zeros(t_t,3);
for i=1:t_t
    position_o(i,:)=CW_Matrix_pos(t_t_vector(i),position(time,:),delta_v_t(time,:)-velocity_o(time,:),n_t);
    velocity_o(i,:)=CW_Matrix_velo(t_t_vector(i),position(time,:),delta_v_t(time,:)-velocity_o(time,:),n_t);
end
total_distance=sqrt(position(:,1).^2+position(:,2).^2+position(:,3).^2);
total_distance_o=sqrt(position_o(:,1).^2+position_o(:,2).^2+position_o(:,3).^2);
%%
plot3(position(:,1),position(:,2),position(:,3))
hold on
%plot3(position_standby(:,1),position_standby(:,2),position_standby(:,3))
plot3(position_o(:,1),position_o(:,2),position_o(:,3))
plot3(0,0,0,".")
plot3(position(time,1),position(time,2),position(time,3),'o')
plot3(R,T,N)
legend("Orbit of chaser without control","Orbit with control", "position of target","Postion of transfer","ideal formation")
xlabel("Radial (km)")
ylabel("Tangential (km)")
zlabel("Normal (km)")
title("The orbital position of the chaser around the target")
hold off
%%
figure;
subplot(2, 1, 1);
plot(t,velocity(:,1))
hold on
plot(t,velocity(:,2))
plot(t,velocity(:,3))
xlabel("Time (sec)")
ylabel("velocity (km/s)")
legend("Radial","Tangential","Normal")
title('Relative Velocity of the Chaser')
hold off
subplot(2, 1, 2);
plot(t,position(:,1))
hold on
plot(t,position(:,2))
plot(t,position(:,3))
xlabel("Time (sec)")
ylabel("distance (km)")
legend("Radial","Tangential","Normal")
title('Relative Position of the Chaser')
hold off
%%
figure;
subplot(2, 1, 1);
plot(t_t_vector,velocity_o(:,1))
hold on
plot(t_t_vector,velocity_o(:,2))
plot(t_t_vector,velocity_o(:,3))
xlabel("Time (sec)")
ylabel("velocity (km/s)")
legend("Radial","Tangential","Normal")
title('Relative Velocity of the Chaser')
hold off
subplot(2, 1, 2);
plot(t_t_vector,position_o(:,1))
hold on
plot(t_t_vector,position_o(:,2))
plot(t_t_vector,position_o(:,3))
xlabel("Time (sec)")
ylabel("distance (km)")
legend("Radial","Tangential","Normal")
title('Relative Position of the Chaser')
hold off
%% CW equations

function position=CW_Matrix_pos(t,r0,v0,n)
f=n*t;
phi_rr=[4-3*cos(f),     0,   0;
        6*(sin(f)-f),   1,   0;
        0               0,   cos(f)];
phi_rv=[1/n*sin(f),      2/n*(1-cos(f)),   0;
        2/n*(cos(f)-1),  1/n*(4*sin(f)-3*f), 0;
        0,               0,                sin(f)/n];
position=phi_rr*r0.'+phi_rv*v0.';
end
function velocity=CW_Matrix_velo(t,r0,v0,n)
f=n*t;
phi_vr=[3*n*sin(f),     0,    0;
        6*n*(cos(f)-1), 0,    0;
        0,              0,    -n*sin(f)];
phi_vv=[cos(f),    2*sin(f),    0;
        -2*sin(f),  4*cos(f)-3, 0;
        0,          0,           cos(f)];
velocity=phi_vr*r0.'+phi_vv*v0.';
end

function C=COE_To_ROE(big_omega,small_omega,theta,inclination)
c1=cos(big_omega);
c3=cos(small_omega+theta);
c2=cos(inclination);
s1=sin(big_omega);
s3=sin(small_omega+theta);
s2=sin(inclination);
C=[c1*c3-s1*c2*s3, -c1*s3-s1*c2*c3, s1*s2;
    s1*c3+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2;
    s2*s3,           s2*c3,           c2];
end

function [H,i,e,big_omega,small_omega,theta]=COE(r,v,mu)
%Z=[0,0,r(3)];
%X=[r(1),0,0];
R=norm(r);
V=norm(v);
vr=dot(r,v)/R;
h=cross(r,v);
H=norm(h);
i=acos(h(3)/H);

K=[0,0,1];
n=cross(K,h);
N=norm(n);
if n(2)>=0
    big_omega=acos(n(1)/N);
else
    big_omega=2*pi-acos(n(1)/N);
end
E=1/mu*((V^2-mu/R).*r-R*vr.*v);
e=sqrt(1+H^2/(mu^2)*(V^2-2*mu/R));

if E(3)>=0
    small_omega=acos(dot(n,E)/(N*e));
else
    small_omega=2*pi-acos(dot(n,E)/(N*e));
end
if dot(r,v)>=0
    theta=acos(dot(E,r)/(e*R));
else
    theta=2*pi-acos(dot(E,r)/(e*R));
end
end

function v_t0=transfer_to_observation(t,delta_r,delta_r0,n)
f=n*t;
phi_rv=[1/n*sin(f),      2/n*(1-cos(f)),   0;
        2/n*(cos(f)-1),  4/n*(sin(f)-3*f), 0;
        0,               0,                sin(f)/n];

phi_rr=[4-3*cos(f),     0,   0;
        6*(sin(f)-f),   1,   0;
        0               0,   cos(f)];
v_t0=(inv(phi_rv)*(delta_r.'-phi_rr*delta_r0.')).';
end

