clear, close all; clc

%% Excerice 3.31
Fe = [0;0;0;0;0;10];
Tce = [0 0 1 -75; -1/sqrt(2) 1/sqrt(2) 0 -260/sqrt(2); ...
       -1/sqrt(2) -1/sqrt(2) 0 130/sqrt(2); 0 0 0 1]
Tec = Tce^-1
Pec = Tec(1:3,4);
Pecx = [0 -Pec(3) Pec(2); Pec(3) 0 -Pec(1); -Pec(2) Pec(1) 0];
AdTec = [Tec(1:3,1:3), zeros(3,3); Pecx*Tec(1:3,1:3), Tec(1:3,1:3)]
Fc = AdTec'*Fe

%% Exercise 4.14
addpath('MC MATLAB/ModernRobotics-master/packages/MATLAB/mr')

M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
Slist = [[0; 0;  1;  4; 0;    0], ...
       [0; 0;  0;  0; 1; 0], ...
       [0; 0; -1; -6; 0; 0]];
thetalist =[pi / 2; 3; pi];
fprintf('Space: \n')
T = FKinSpace(M, Slist, thetalist)

Blist = [[0; 0; -1; 2; 0; 0], [0; 0; 0; 0; 1; 0], [0; 0; 1; 0; 0; 0]];
fprintf('Body: \n')
T = FKinBody(M, Blist, thetalist)
    