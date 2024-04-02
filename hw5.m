clear, close all; clc;
addpath('MC MATLAB/ModernRobotics-master/packages/MATLAB/mr')
%% Corrections
%% Exercise 5.11
%% a) 
syms L
% Slist = [[0; 0;  1;  0; 0;    0], ...
%        [0; -1;  0;  0; 0; -2*L], ...
%        [0; -1; 0; -1*L; 0; -2*L]];
% thetalist = [0; 0; 0];
% Js = Slist;
% T = eye(4);
% for i = 2: length(thetalist)
%     T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
% 	Js(:, i) = Adjoint(T) * Slist(:, i);
% end
% Js
% V = [0;0;0;10;0;0];
% thetadot = pinv(Js)*V
% Solved for Js instead of Jb
%Not supposed to use pinv but linear component of Jb
%% a) correction
Blist = [[0;0;1;cross([0;0;1],-[-2*L;-L;-L])]...
         [0;-1;0;cross([0;-1;0],-[-2*L;0;0])]...
         [0;-1;0;cross([0;-1;0],-[-L;0;0])]];
Jb = Blist;
T = eye(4);
thetalist = [0; 0; 0];
for i = length(thetalist) - 1: -1: 1   
    T = T * MatrixExp6(VecTose3(-1 * Blist(:, i + 1) * thetalist(i + 1)));
	Jb(:, i) = Adjoint(T) * Blist(:, i);
end
simplify(Jb)
V = [10;0;0];
thetadot = inv(Jb(4:6,1:end))*V
%No solution

%% b)
thetalist = deg2rad([0; 45; -45]);
Fb = [0;0;0;10;0;0];
% Js = Slist;
% T = eye(4);
% for i = 2: length(thetalist)
%     T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
% 	Js(:, i) = Adjoint(T) * Slist(:, i);
% end
% Tsb = [1 0 0 2; 0 1 0 1; 0 0 1 2; 0 0 0 1];
% Jb = Adjoint(Tsb)*Js
% torque = simplify(Jb'*Fb)
%wrong Jb matrix
%% b) correction
Blist = [[0;0;1;cross([0;0;1],-[-2*L;-L;-L])]...
         [0;-1;0;cross([0;-1;0],-[-2*L;0;0])]...
         [0;-1;0;cross([0;-1;0],-[-L;0;0])]];
Jb = Blist;
T = eye(4);
for i = length(thetalist) - 1: -1: 1   
    T = T * MatrixExp6(VecTose3(-1 * Blist(:, i + 1) * thetalist(i + 1)));
	Jb(:, i) = Adjoint(T) * Blist(:, i);
end
simplify(Jb)
torque = simplify(Jb'*Fb)
%% c)
Fb = [10;0;0;0;0;0];
torque = Jb'*Fb
% correct

%% d)
% torque = [10;20;5];
% F = simplify(abs(pinv(Jb')*torque))
% wrong angles for Jb
% don't use pinv solve for fx symbolicly
%% d) correction
Blist = [[0;0;1;cross([0;0;1],-[-2*L;-L;-L])]...
         [0;-1;0;cross([0;-1;0],-[-2*L;0;0])]...
         [0;-1;0;cross([0;-1;0],-[-L;0;0])]];
Jb = Blist;
T = eye(4);
thetalist = [0; 0; 0];
for i = length(thetalist) - 1: -1: 1   
    T = T * MatrixExp6(VecTose3(-1 * Blist(:, i + 1) * thetalist(i + 1)));
	Jb(:, i) = Adjoint(T) * Blist(:, i);
end
simplify(Jb);
syms fx;
F = [0;0;0;fx;0;0];
Jb'*F

%% Exercise 5.17
clear all;
%% a)
syms L t1 t2 t3 t4 t5 t6
Slist = [[ 0; 0; 0; 0; 0; 1], ...
         [ 1; 0; 0; 0; 0; 0], ...
         [ 0; 0; 1; cross([0;0;1],-[0;L;0])], ...
         [ 1; 0; 0; cross([1;0;0],-[L;L;0])], ...
         [cosd(45);sind(45);0;cross([cosd(45);sind(45);0],-[L;2*L;0])], ...
         [ 0; 0; 0; 0; 1; 0]];
thetalist = [t1, t2, t3, t3, t5, t6];
Js = Slist;
T = eye(4);
for i = 2: length(thetalist)
    se3mat = VecTose3(Slist(:, i - 1) * thetalist(i - 1));
    omgtheta = so3ToVec(se3mat(1: 3, 1: 3));
    if norm(omgtheta) == 0
        T = [eye(3), se3mat(1: 3, 4); 0, 0, 0, 1];
    else
        [omghat, theta] = AxisAng3(omgtheta);
        omgmat = se3mat(1: 3, 1: 3) / theta; 
        R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
        T = [R, ...
             (eye(3) * theta + (1 - cos(theta)) * omgmat ...
              + (theta - sin(theta)) * omgmat * omgmat) ...
                * se3mat(1: 3, 4) / theta;
             0, 0, 0, 1];
    end
	Js(:, i) = simplify(Adjoint(T) * Slist(:, i));
end
fprintf("First 3 columns of Js are:");
Js(:,1:3)
% no simplification however this Js works for part b and c
%% b)
thetalist = [0,0,0,0,0,0];
thetadot = [1;0;1;-1;2;0];
T = eye(4);
for i = 2: length(thetalist)
    T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
	Js(:, i) = Adjoint(T) * Slist(:, i);
end
Js
Vs = Js*thetadot
%correct

%% c)
rank(Js)
size(Js,2)

%% Exercise 5.25
clear all;
%% a)
W1 = 109; W2 = 82; L1 = 425; L2 = 392; H1 = 89; H2 = 95;
fprintf("joint angles are:")
thetalist = pi/2*ones(1,6)
Slist = [[0;0;1;0;0;0], ...
         [0;1;0;-H1;0;0], ...
         [0;1;0;-H1;0;L1], ...
         [0;1;0;-H1;0;L1+L2], ...
         [0;0;-1;-W1;L1+L2;0], ...
         [0;1;0;H2-H1;0;L1+L2]];
Js = Slist;
T = eye(4);
for i = 2: length(thetalist)
    T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
	Js(:, i) = Adjoint(T) * Slist(:, i);
end
fprintf("Js is:")
Js
fprintf("Jw is:")
Jw = Js(1:3,1:end)
fprintf("Jv is:")
Jv = Js(4:6,1:end)
%correct
%% b)
Aw = Jw*Jw';
Av = Jv*Jv';
[Vw, Dw] = eig(Aw);
[Vv, Dv] = eig(Av);
[xw,yw,zw] = ellipsoid(0,0,0,sqrt(Dw(1,1)),sqrt(Dw(2,2)),sqrt(Dw(3,3)));
[xv,yv,zv] = ellipsoid(0,0,0,sqrt(Dv(1,1)),sqrt(Dv(2,2)),sqrt(Dv(3,3)));
thetaw = acos(0.5*(trace(Vw)-1));
omegaw = so3ToVec(MatrixLog3(Vw));
thetav = acos(0.5*(trace(Vv)-1));
omegav = so3ToVec(MatrixLog3(double(Vv)));
figure(1)
Sw = surf(double(xw),double(yw),double(zw));
rotate(Sw,omegaw',rad2deg(thetaw))
title("3D angular-velocity manipulability ellipsoid")
figure(2)
Sv = surf(double(xv),double(yv),double(zv));
rotate(Sv,omegav',rad2deg(thetav))
title("3D linear-velocity manipulability ellipsoid")

% got the right Aw and Av however it is hard to tell if the ellipsoids are
% correct from the plots given in solutions
%% c)
Jw = Js(1:3,1:end);
Jv = Js(4:6,1:end);
Bw = (Jw*Jw')^-1;
Bv = (Jv*Jv')^-1;
[Vw, Dw] = eig(Bw);
[Vv, Dv] = eig(Bv);
fprintf("Force ellipsoid Jw \n")
fprintf("Lengths: ")
sqrt(diag(Dw)')
fprintf("Directions: ")
Vw
fprintf("Force ellipsoid Jv \n")
fprintf("Lengths: ")
sqrt(diag(Dv)')
fprintf("Directions: ")
Vv

%right lengths and directions just with different order for
%eigenvectors/values
%also lengths are scaled based on mm not m as in solutions


