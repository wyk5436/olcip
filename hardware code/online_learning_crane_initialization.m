clc


m = 0.114; % kg
l = 0.33; % m
r = 0.22; % m
K1 = 0.2065;
K2 = 0.1105;
J = 0.0076;
g = 9.8;
dt = 0.01;

% CRANE CONTROL GAINS
% linearize about the pendulum DOWN position
Ad = [0 0 1 0; 0 0 0 1; 0 m*g*r/J -K2/J 0; 0 -(J+m*r^2)*g/(J*l) r*K2/(J*l) 0];
Bd = [0 0 K1/J -r*K1/(J*l)]';
Cd = [1 0 0 0];

sys = ss(Ad,Bd,Cd,0);

dis_sys = c2d(sys,dt);
dis_A = dis_sys.A;
dis_B = dis_sys.B;

Q_cr = diag([5 30 0 0]);
R_cr = 1;

K_cr = dlqr(dis_A, dis_B, Q_cr, R_cr);

%%
t = alpha.time(60:end);
dt = t(2) - t(1);
m = length(t);
alpha_data = alpha.signals.values(60:end);
theta_data = theta.signals.values(60:end);
alphadot_data = alphadot.signals.values(60:end);
thetadot_data = thetadot.signals.values(60:end);
Z = [alpha_data';theta_data';alphadot_data';thetadot_data'];
X = Z(:,1:m-1);
Y = Z(:,2:end);
Am = Y*pinv(X);

Am = logm(Am)/dt;
Km = lqr(Am, Bd, Q_cr, R_cr);