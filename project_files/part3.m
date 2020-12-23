clc
clear
cd '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project/project_files/'
tbxmanager restorepath
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/casadi-osx-matlabR2015a-v3.5.5'
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project'
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project/src/'
import casadi.*
%%

clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SYSTEM DEFINITION AND LINEARIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

quad = Quad();
[xs, us] = quad.trim();
sys = quad.linearize(xs,us);

% SYSTEM TRANSFORMATION : 
sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTING CONTROLLERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = 1/5;
Tf = 10.0;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

clc
mpc_x = MPC_Control_x(sys_x, Ts); 
mpc_y = MPC_Control_y(sys_y, Ts);  
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SYSTEM SIMULATION FOR INDIVIDUAL DIMENSIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = 100;

 x(:,1) = [0; 0; 0; 1]; %Initial state
 y(:,1) = [0; 0; 0; 1]; %Initial state
 z(:,1) = [0; 1];
 yaw(:,1) = [0; 2];
 
 
t_plot = [0:Ts:N*Ts];

for i = 1:N
    
     ux(:,i) = mpc_x.get_u(x(:,i));
     uy(:,i) = mpc_y.get_u(y(:,i));
     uz(:,i) = mpc_z.get_u(z(:,i));
     uyaw(:,i) = mpc_yaw.get_u(yaw(:,i));
     
     x(:,i+1) = mpc_x.A * x(:,i) + mpc_x.B * ux(:,i);
     y(:,i+1) = mpc_y.A * y(:,i) + mpc_y.B * uy(:,i);
     z(:,i+1) = mpc_z.A * z(:,i) + mpc_z.B * uz(:,i);
     yaw(:,i+1) = mpc_yaw.A * yaw(:,i) + mpc_yaw.B * uyaw(:,i);
     
 end

subplot(2,2,1)
plot(t_plot,x(1,:),t_plot,x(2,:),t_plot,x(3,:),t_plot,x(4,:))
legend('vel pitch [rad/s]', 'pitch [rad]', 'vel x [m/s]', 'x [m]','Location','east');
title('x states');
xlabel('time [s]');

subplot(2,2,2)
plot(t_plot,y(1,:),t_plot,y(2,:),t_plot,y(3,:),t_plot,y(4,:))
legend('vel pitch [rad/s]', 'roll [rad]', 'vel y [m/s]', 'y [m]','Location','east');
title('y states');
xlabel('time [s]');

subplot(2,2,3)
plot(t_plot,z(1,:),t_plot,z(2,:))
legend('vel z [m/s]', 'z [m]');
title('z states');
xlabel('time [s]');
 
subplot(2,2,4)
plot(t_plot,yaw(1,:),t_plot,yaw(2,:))
legend('vel yaw [rad/s]', 'yaw [rad]');
title('z states');
xlabel('time [s]');
 
 
 

%%

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller
mpc_x = MPC_Control_x(sys_x, Ts);
% Get control inputs with
ux = mpc_x.get_u(x)

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim = quad.sim(mpc_x,mpc_y,mpc_z,mpc_yaw);
quad.plot(sim);




