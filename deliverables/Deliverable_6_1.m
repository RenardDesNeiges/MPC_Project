clc
clear
close all

addpath('casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NMPC SYSTEM DEFINITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%system and controller definition
quad = Quad();
MPC = ctrl_NMPC(quad);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NMPC TRACKING PROBLEM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% simulating the tracking problem and plotting the result
sim = quad.sim(MPC);
quad.plot(sim);
set(gcf,'color','w');

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NMPC RECOVERY EXAMPLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%system and controller definition
quad = Quad();
MPC = ctrl_NMPC(quad);

%   saving coordinates from a simulated run in order to use them 
%   in the plots below
[thN,xhN,uhN,rhN] = saveRunRecover(quad, MPC);

%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS FOR THE NMPC RECOVERY EXAMPLE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Trace plot

close all

figure(1)

clf; view(3);
hold on; grid on;

pos = xhN(10:12,:);
plot3(pos(1,:), pos(2,:), pos(3,:), 'g', 'linewidth', 1);



for i = 1:5:70
    quad.plot_point(xhN(:,i),uhN(:,i));
end
    
title('Successful recovery from inverted position')
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
    
axis equal
axis vis3d
set(gcf,'color','w');
    
xlim([-20 20])
ylim([-80 10])
zlim([-120 30])

%% Attitude plots

close all

figure(1)

subplot(2,3,1)
quad.plot_point(xhN(:,1),uhN(:,1));
title("Drone attitude a t = 0s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

subplot(2,3,2)
quad.plot_point(xhN(:,5),uhN(:,5));
title("Drone attitude a t = 1s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

subplot(2,3,3)
quad.plot_point(xhN(:,10),uhN(:,10));
title("Drone attitude a t = 2s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

subplot(2,3,4)
quad.plot_point(xhN(:,15),uhN(:,15));
title("Drone attitude a t = 3s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

subplot(2,3,5)
quad.plot_point(xhN(:,20),uhN(:,20));
title("Drone attitude a t = 4s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

subplot(2,3,6)
quad.plot_point(xhN(:,25),uhN(:,25));
title("Drone attitude a t = 5s")
axis equal
axis vis3d
xlabel('x')
ylabel('y')
zlabel('z')

set(gcf,'color','w');

