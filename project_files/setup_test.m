clc
clear
cd '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project/project_files/'
tbxmanager restorepath
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/casadi-osx-matlabR2015a-v3.5.5'
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project'
addpath '/Users/renard/Documents/etudes/EPFLMA1/MPC/MPC_Project/src/'
import casadi.*
%%
disp("testing casadi")
x = MX.sym('x')
disp(jacobian(sin(x),x))

%%
disp("testing gurobi")
N = 10 %constraints
n = 5 %variables
A = sprandn(N,n,0.5); 
imagesc(A)
b = ones(N,1); 
x0 = 10*randn(n,1);
x = sdpvar(n,1);
opt = optimize(A*(x - x0) <= b, x'*x)


%%

disp("testing quad")
quad = Quad()

x0 = zeros(12,1); % Initial state
u = [1;1;1;.4];

Tf = 5.0; % Time to simulate for

sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0)

quad.plot(sim, u);

%quad.f(x,u)

%