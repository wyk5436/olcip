function f = nonlinear_crane(t,x,u)
    m = 0.114;
    J = 0.0076;
    l = 0.337;
    r = 0.216;
    g = 9.81;
    K1 = 0.2065;
    K2 = 0.1105;

    alpha = x(1);
    theta = x(2);
    alphad = x(3);
    thetad = x(4);

    alpha_dot = x(3);
    theta_dot = x(4);

    tau = K1*u - K2*alpha_dot;

    denom = J + m*r^2 + m*l^2*sin(theta)^2-m*r^2*cos(theta)^2;
    alpha_ddot = tau-(m*r*l*cos(theta)*(alphad^2*sin(theta)*cos(theta)-g*sin(theta)/l)+m*r*l*thetad^2*sin(theta) - 2*m*l^2*alphad*thetad*sin(theta)*cos(theta)) / denom;

    K = J + m*r^2+m*l^2*sin(theta)^2;
    thetaddot_numer = -(r/l)*(tau + m*r*l*thetad^2*sin(theta)-2*m*l^2*alphad*thetad*sin(theta)*cos(theta))*cos(theta)/K + alphad^2*sin(theta)*cos(theta)-g*sin(theta)/l;
    thetaddot_denom = 1 + m*r^2*cos(theta)^2/K;
    theta_ddot = thetaddot_numer/thetaddot_denom;
    f = [alpha_dot;theta_dot;alpha_ddot;theta_ddot];
end