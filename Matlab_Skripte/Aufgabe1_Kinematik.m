%-------------------------------------------------------------------------------------%
%MSM-Weihnachtsprojekt
%Andreas Paul
%Mat.:3418246
%06.01.2025
%-------------------------------------------------------------------------------------%
clc;
clear;
close all;
%Transformationsmatrizen für die Kinematik des Roboters

windows_ = ~(isunix);

%Verallgemeinerte Koordinaten:
syms alpha beta

%für Jacobians
syms alpha_dot beta_dot
y = [alpha; beta];
y_dot = [alpha_dot; beta_dot];

%Roboterparameter:
syms l1 l2

%Berechung der Transformationsmatritzen
T_01 = [sin(alpha), -cos(alpha), 0, 0;
        cos(alpha),  sin(alpha), 0, 0;
        0,           0,          1, 0;
        0,           0,          0, 1];

if windows_
matlabFunction(T_01,'File', 'D:\MASTER\Semester3\MSM\Weihnachtsprojekt\Matlab_Skripte\Systemmatrizen\forwardKinematics\calc_T_01', ...
               'Vars', {y,l1,l2});
else
matlabFunction(T_01,'File', 'Systemmatrizen/forwardKinematics/calc_T_01', ...
               'Vars', {y,l1,l2});
end

T_12_trans = [1, 0, 0, l1;
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];  %in KOS 1
              
T_12_rot =  [sin(beta), -cos(beta), 0, 0;
             cos(beta),  sin(beta), 0, 0;
             0,          0,         1, 0;
             0,          0,         0, 1];   %rot um z1-Achse

T_12 = T_12_trans * T_12_rot;

T_02 = T_01 * T_12;

if windows_
matlabFunction(T_02,'File', 'D:\MASTER\Semester3\MSM\Weihnachtsprojekt\Matlab_Skripte\Systemmatrizen\forwardKinematics\calc_T_02', ...
               'Vars', {y,l1,l2});
else
matlabFunction(T_02,'File', 'Systemmatrizen/forwardKinematics/calc_T_02', ...
               'Vars', {y,l1,l2});
end

T_23 = [1, 0, 0, l2;
        0, 1, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1];  %=T_2EF; in KOS 2 

T_0EF = T_01 * T_12 * T_23;  %POSE Endeffektor

disp('T_0EF = ');
disp(T_0EF);

if windows_
matlabFunction(T_0EF,'File', 'Matlab_Skripte\Systemmatrizen\forwardKinematics\calc_T_0EF', ...
               'Vars', {y,l1,l2});
else
matlabFunction(T_0EF,'File', 'Systemmatrizen/forwardKinematics/calc_T_0EF', ...
               'Vars', {y,l1,l2});
end

%-------------------------------------------------------------------------------------%
%Man kann die symbolic Toolbox nicht in Simulink verwenden...
%-------------------------------------------------------------------------------------%

%exportiere J, J_dot --> Gedankengang siehe InverseKinematics_TESTFILE.m
r = T_0EF(1:3,4);

%Geschwindigkeit
%Notiz: r ist in inertial coordiantes? KA deutsch lol, deswegen physical
%derr. gleich numerischer derr. 
%und nutze Kettenregel: r_dot = (dr/dy)*y_dot
J = jacobian(r,y);

%Beschleunigung
%erneut Kettenregel:
%r_ddot = J_dot*y_dot + J*y_ddot
%Da J eine Matrix ist, kann man nicht einfach mit jacobian Befehl arbeiten,
%muss Elementweise erledigt werden.
J_dot = sym(zeros(size(J))); 
for i = 1:size(J, 1)
    for j = 1:size(J, 2)
        J_dot(i, j) = jacobian(J(i, j), y) * y_dot;
    end
end

%Exportieren als Funktionen
if windows_
matlabFunction(J,'File', 'Matlab_Skripte\Systemmatrizen\forwardKinematics\calc_J', ...
               'Vars', {y, l1, l2});
matlabFunction(J_dot,'File', 'Matlab_Skripte\Systemmatrizen\forwardKinematics\calc_J_dot', ...
               'Vars', {y, y_dot, l1, l2});
else
matlabFunction(J,'File', 'Systemmatrizen/forwardKinematics/calc_J', ...
               'Vars', {y, l1, l2});
matlabFunction(J_dot,'File', 'Systemmatrizen/forwardKinematics/calc_J_dot', ...
               'Vars', {y, y_dot, l1, l2});
end


%---------------------TEST DH----------------------------%
% Define the modified DH parameters
alpha1 = 0; a1 = 0; d1 = 0; theta1 = alpha;
alpha2 = 0; a2 = l1; d2 = 0; theta2 = beta;
alpha3 = 0; a3 = l2; d3 = 0; theta3 = 0;

% Transformation from frame 0 to frame 1
T_01_DH = [sin(theta1), -cos(theta1), 0, a1;
           cos(theta1),  sin(theta1), 0, 0;
           0,            0,           1, d1;
           0,            0,           0, 1];

% Transformation from frame 1 to frame 2
T_12_DH = [sin(theta2), -cos(theta2), 0, a2;
           cos(theta2),  sin(theta2), 0, 0;
           0,            0,           1, d2;
           0,            0,           0, 1];

% Transformation from frame 2 to frame 3
T_23_DH = [1, 0, 0, a3;
           0,  1, 0, 0;
           0,            0,           1, d3;
           0,            0,           0, 1];

% Compute the overall transformation from frame 0 to end-effector
T_0EF_DH = T_01_DH * T_12_DH * T_23_DH;

% Compute the difference between the actual and DH-based transformations
delta_DH = T_0EF - T_0EF_DH;
disp('delta_DH = ');
disp(delta_DH);
