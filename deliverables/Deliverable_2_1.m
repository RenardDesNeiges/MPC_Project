clc
clear
close all

addpath('casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

%% DELIVERABLE 2.1 %%
quad = Quad();
[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v