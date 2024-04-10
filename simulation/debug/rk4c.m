function yy = rk4c(t,h,y,K,f)
    K1 = f(t,y,-K*y);
    K2 = f(t + h/2,y + h*K1/2,-K*(y + h*K1/2));
    K3 = f(t + h/2,y + h*K2/2,-K*(y + h*K2/2));
    K4 = f(t + h, y + h*K3, -K*(y + h*K3));
    yy = y + (h/6)*(K1 + 2*K2 + 2*K3 + K4);
end