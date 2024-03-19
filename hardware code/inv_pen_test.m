clc
clear all


m = 0.114; % kg
l = 0.33; % m
r = 0.22; % m
K1 = 0.2065;
K2 = 0.1105;
J = 0.0076;
g = 9.8;

% CRANE CONTROL GAINS
% linearize about the pendulum DOWN position
Ad = [0 0 1 0; 0 0 0 1; 0 m*g*r/J -K2/J 0; 0 (J+m*r^2)*g/(J*l) -r*K2/(J*l) 0];
Bd = [0 0 K1/J r*K1/(J*l)]';

Q_cr = diag([3 50 0 0]);
R_cr = 1;

K_cr = lqr(Ad, Bd, Q_cr, R_cr);

