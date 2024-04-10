clc
clear all

% default plot attributes
set(0, 'defaultFigurePosition',  [10  10  1200 800])
set(0,'defaultaxesfontname','times');
set(0,'defaultaxesfontsize',20);

x0 = [-0.239284845902261;-0.367616812291287;0.939753669149791;1.70274666282027];
T_offset = 2.01;
n_observe = 99;
dt = 0.01;
t_observe = 0:dt:n_observe*dt;
t_observe = T_offset + t_observe;

dxdt_handle = @nonlinear_crane;
full_state = zeros(4,n_observe + 1);
full_state(:,1) = x0;

m = 0.114; % kg
l = 0.33; % m
r = 0.22; % m
K1 = 0.2065;
K2 = 0.1105;
J = 0.0076;
g = 9.8;
dt = 0.01;

A = [0 0 1 0; 0 0 0 1; 0 m*g*r/J -K2/J 0; 0 -(J+m*r^2)*g/(J*l) r*K2/(J*l) 0];
B = [0 0 K1/J -r*K1/(J*l)]';
disc_A = expm(A*dt);
disc_B = dt*B;
Q = diag([5 30 0 0]);
R = 1;
K = [0 0 0 0];
K_acc = lqr(A,B,Q,R);

for i = 1:n_observe
    old_state = full_state(:,i);
    full_state(:,i+1) = rk4c(t_observe(i),dt,old_state,K,dxdt_handle);
end

DMD_err = zeros(1,n_observe);
for i = 1:n_observe
    X = full_state(:,1:i);
    Y = full_state(:,2:i+1);
    At = Y*pinv(X);
    DMD_err(i) = norm(At-disc_A)/norm(disc_A);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now entering: CONTROL!!!!!!!!!!!!!!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Am = 0;
P = 0;
xkp1 = 0;

[Am,P,xkp1] = init_dmd(full_state);
K = dlqr(Am,disc_B,Q,R);


n_ctrl = 399;
t_ctrl = 0:dt:n_ctrl*dt;
t_ctrl = t_ctrl + t_observe(end);
new_state = zeros(4,n_ctrl + 1);
new_state(:,1) = full_state(:,end);

uk = -K*full_state(:,end);

A_err = zeros(1,n_ctrl);
A_err(1) = norm(disc_A-Am)/norm(disc_A);

for i = 1:n_ctrl
    old_state = new_state(:,i);
    update = rk4c(t_ctrl(i),dt,old_state,K,dxdt_handle);
    new_state(:,i+1) = update;

    [At,Pt,xkp1t] = online_dmd_update(Am,P,xkp1,update,uk);
    K = dlqr(Am,disc_B,Q,R);

    Am = At;
    P = Pt;
    xkp1 = xkp1t;
    uk = -K*update;

    A_err(i+1) = norm(disc_A-Am)/norm(disc_A);
end

%%%%%%%%%%%%%%%%%%%%%%
% reference trajectory
%%%%%%%%%%%%%%%%%%%%%%
ref_state = zeros(4,n_ctrl + 1);
ref_state(:,1) = full_state(:,end);
for i = 1:n_ctrl
    ref_state(:,i+1) = rk4c(t_ctrl(i),dt,ref_state(:,i),K_acc,dxdt_handle);
end

%%%%%%%%%%%%%%%%%%%%%%%
% plotting
%%%%%%%%%%%%%%%%%%%%%%%
t = [t_observe,t_ctrl(2:end)];
all_state = [full_state,new_state(:,2:end)];
all_state_acc = [full_state,ref_state(:,2:end)];
theta = all_state(2,:);
theta_acc = all_state_acc(2,:);

total_err = [inf DMD_err A_err(2:end)];

figure;
plot(t,theta,t,theta_acc);
xline(3,'--k', 'LineWidth',1)
xlabel("time");
ylabel('\theta');
legend("Learned A", "Accurate A")

figure;
plot(t,total_err);
xlabel("time");
ylabel('percentage error');


function [Am,P,xkp1] = init_dmd(data)
    i = length(data);
    X = data(:,1:i-1);
    Y = data(:,2:end);
    
    xkp1 = Y(:,end);
    Am = Y*pinv(X);
    P = inv(X*X');
end

function [At,Pt,xkp1t] = online_dmd_update(A,P,xkp1,x,uk) 
    %{
    A: A matrix at the previous time step
    xkp1: x_{k+1} paramete for online DMD.
    P: P matrix at the previous time step   
    uk: the control at the previous step.
    x: The state at this time step, resulting from uk
    %}

    K1 = 0.2065;
    J = 0.0076;
    l = 0.337;
    r = 0.216;
    B = [0; 0; K1/J;-r*K1/(J*l)];
    dt = 0.01;
    disc_B = B*dt;

    gamma = 1/(1 + xkp1'*P*xkp1);
    ykp1 = x - disc_B*uk;
    
    At = A + gamma*(ykp1 - A*xkp1)*xkp1'*P;
    Pt = P - gamma*P*(xkp1*xkp1')*P;
    xkp1t = x;
end