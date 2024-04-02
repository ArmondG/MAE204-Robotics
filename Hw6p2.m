clear all; clc;
addpath('MC MATLAB/ModernRobotics-master/packages/MATLAB/mr')
%% Corrections
%lengths in m
W1 = 109/1000;
W2 = 82/1000;
L1 = 425/1000;
L2 = 392/1000;
H1 = 89/1000;
H2 = 94/1000;
%Get e-e frame at zero position
M = [[-1, 0, 0, L1+L2]; [0, 0, 1, W1+W2]; [0, 1, 0, H1-H2]; [0, 0, 0, 1]];
%Calculate Blist
Slist = [[0;0;1;0;0;0],[0;1;0;-H1;0;0],[0;1;0;-H1;0;L1],...
         [0;1;0;-H1;0;L1+L2],[0;0;-1;-W1;L1+L2;0],[0;1;0;H2-H1;0;L1+L2]];
Blist = zeros(size(Slist));
for i = 1:size(Slist,2)
    Blist(:,i) = Adjoint(M^-1)*Slist(:,i);
end
%Provided e-e configuration and error tolerances
T = [[0, 1, 0, -0.5]; [0, 0, -1, 0.1]; [-1, 0, 0, 0.1]; [0, 0, 0, 1]];
eomg = 0.001;
ev = 0.001;
%Initial guess of thetalist using visualization from Coppelia
thetalist0 = [0;-2.049;-1.587;-2.710;3.173;-1.571];
%IKinBodyIterations
[thetalist, success] = ...
    IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev)

%% Seems correct when compared with solutions