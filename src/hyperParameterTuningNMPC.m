%%

clc
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SYSTEM DEFINITION FOR NMPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

quad = Quad();

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SCORING A RUN (AND TIMING SCORING)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pA = 1;
zA = 100;
yawA = 1;

MPC = ctrl_NMPC(quad, pA, zA, yawA, uA);
tic
[maxE,meanE] = scoreRun(quad, MPC)
toc
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% SCORING SEVERAL RUNS (AND TIMING SCORING)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pA = 1;
yawA = 1;
uA = 0.01;

zAs = []
maxS = []
avgS = []

for i = 0:9
    zA = 10^(i/3.2) * 0.5
   
    MPC = ctrl_NMPC(quad, pA, zA, yawA, uA);
    tic
    [maxE,meanE] = scoreRun(quad, MPC);
    toc
    
    zAs = [ zAs, zA ];
    maxS = [ maxS, maxE ];
    avgS = [ avgS, meanE ];
end

%%

function [maxE,meanE] = scoreRun(quad,MPC) 
    [th,xh,uh,rh] = saveRun(quad, MPC);
    sqex = ( rh(1,:)- xh(10,:) ).^2;
    sqey = ( rh(2,:)- xh(11,:) ).^2;
    sqez = ( rh(3,:)- xh(12,:) ).^2;
    sqeyaw = ( rh(4,:)- xh(6,:) ).^2;
    sqe = sum( [sqex;sqey;sqez;sqeyaw] ,1).^0.5 ;
    maxE = max(sqe);
    meanE = mean(sqe);
end