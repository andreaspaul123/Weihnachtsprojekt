clc;
clear;
close all;

windows_ = ~(isunix);
addpath(genpath('Systemmatrizen'))
%param def
l1 = 0.16;
l2 = 0.128;
syms alpha beta t
y = [alpha; beta];

%import POSE of EF and jacobians
T_0EF = calc_T_0EF(y,l1,l2);

%Position
r = T_0EF(1:3,4);

%get trajektorie
tra = calc_Trajektorie_kartesisch(t);
rd = tra(1:3);
%% Berechne yd

%Berechne yd
t_ = 0:0.05:2;
t_steps = length(t_);
yd_alpha = zeros(1,t_steps);
yd_beta = zeros(1,t_steps);

for i = 1:t_steps
    rd1 = subs(rd,t,t_(i));
    eq1 = r == rd1;

    yd1 = solve(eq1, alpha, beta);
    alpha_sol = double(yd1.alpha(1));
    beta_sol = double(yd1.beta(1));
    yd_alpha(i) = alpha_sol;
    yd_beta(i) = beta_sol;
end


% Fit a polynomial to the data points
p_alpha = polyfit(t_, yd_alpha, 5); % 5th degree polynomial for yd_alpha
p_beta = polyfit(t_, yd_beta, 5);   % 5th degree polynomial for yd_beta

alpha_des = p_alpha(1) * t^5 + p_alpha(2) * t^4 + p_alpha(3) * t^3 + p_alpha(4) * t^2 +...
    p_alpha(5) * t + p_alpha(6);
beta_des = p_beta(1) * t^5 + p_beta(2) * t^4 + p_beta(3) * t^3 + p_beta(4) * t^2 +...
    p_beta(5) * t + p_beta(6);

yd_alpha_sol = polyval(p_alpha, t_);
yd_beta_sol = polyval(p_beta, t_);

yd = [alpha_des; beta_des];

yd_dot = diff(yd,t);

yd_ddot = diff(yd_dot,t);

x0 = [subs(yd(:,1), t, 0); subs(yd_dot(:,1), t, 0)];

%% EXPORT ALS FUNKTIONEN
Trajectory = [yd; yd_dot; yd_ddot];
if windows_
matlabFunction(Trajectory,'File', ...
    'Matlab_Skripte\Systemmatrizen\Trajektorien\calc_Trajektorie_verallg','Vars', {t});
else
matlabFunction(Trajectory,'File', ...
    'Systemmatrizen/Trajektorien/calc_Trajektorie_verallg','Vars', {t});
end

%% PLOTS

