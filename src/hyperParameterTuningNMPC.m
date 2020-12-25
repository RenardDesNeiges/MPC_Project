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
% SCORING A RUN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


pA = 1;
zA = 100;
yawA = 1;
uA = 0.01;
MPC = ctrl_NMPC(quad, pA, zA, yawA, uA);
[maxE,meanE] = scoreRun(quad, MPC)
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