clc
clear
close all

addpath('casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
%% DELIVERABLE 3.2 %%
 
 Ts       = 1/5;
 quad     = Quad(Ts);
 [xs, us] = quad.trim();
 sys      = quad.linearize(xs, us); 
 [sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
 
 T_set= 8;
 N = (T_set/Ts); %%% p>=(T_set/T_s)
 
% Design MPC controller
 mpc_x   = MPC_Control_x(sys_x, Ts);
 x(:,1) = [0; 0; 0; 0]; %Initial state
 x_ref = -2;
 
% Design MPC controller
 mpc_y   = MPC_Control_y(sys_y, Ts);
 y(:,1) = [0; 0; 0; 0]; %Initial state
 y_ref = -2;
 
%Design MPC controller
 mpc_yaw   = MPC_Control_yaw(sys_yaw, Ts);
 yaw(:,1) = [0; 0]; %Initial state
 yaw_ref = 0.785;
 
 %Design MPC controller
 mpc_z   = MPC_Control_z(sys_z, Ts);
 z(:,1) = [0; 0]; %Initial state
 z_ref = -2;
 
 %Get the states over the horizon
 for i = 1:N
 ux(:,i) = mpc_x.get_u(x(:,i),x_ref);
 x(:,i+1) = mpc_x.A*x(:,i) + mpc_x.B*ux(:,i);

 uy(:,i) = mpc_y.get_u(y(:,i), y_ref);
 y(:,i+1) = mpc_y.A*y(:,i) + mpc_y.B*uy(:,i);      

 uyaw(:,i) = mpc_yaw.get_u(yaw(:,i), yaw_ref);
 yaw(:,i+1) = mpc_yaw.A*yaw(:,i) + mpc_yaw.B*uyaw(:,i);
 
 uz(:,i) = mpc_z.get_u(z(:,i), z_ref);
 z(:,i+1) = mpc_z.A*z(:,i) + mpc_z.B*uz(:,i);  
 end


%Plot x states
t_plot = [0:1/5:8];
figure();
%subplot(2,2,1)
plot(t_plot,x(1,:),t_plot,x(2,:),t_plot,x(3,:),t_plot,x(4,:))
legend('vel pitch [rad/s]', 'pitch [rad]', 'vel x [m/s]', 'x [m]','Location','east');
title('x states');
xlabel('time [s]');

%Plot y states
figure%subplot(2,2,2)
plot(t_plot,y(1,:),t_plot,y(2,:),t_plot,y(3,:),t_plot,y(4,:))
legend('vel roll [rad/s]', 'roll [rad]', 'vel y [m/s]', 'y [m]','Location','east');
title('y states');
xlabel('time [s]');

%Plot yaw states
figure%subplot(2,2,3)
plot(t_plot,yaw(1,:),t_plot,yaw(2,:))
legend('vel yaw [rad/s]', 'yaw [rad]','Location','east');
title('yaw states');
xlabel('time [s]');

%Plot z states
figure%subplot(2,2,4)
plot(t_plot,z(1,:),t_plot,z(2,:))
legend('vel z [m/s]', 'z [m]','Location','east');
title('z states');
xlabel('time [s]');
