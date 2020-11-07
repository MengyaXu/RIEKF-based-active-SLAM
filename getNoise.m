sigma_x = 0.03;
sigma_y = 0.03;
sigma_theta = 0.02;

sigma_r = 0.04;
sigma_beta = 0.04;

sigma_x0 = 0.01;
sigma_y0 = 0.01;
sigma_theta0 = 0.02;

step = 500;
k = 50;

W = [normrnd(0, sigma_theta0) normrnd(0, sigma_theta, 1, step-1); 
    normrnd(0, sigma_x0) normrnd(0, sigma_x, 1, step-1); 
    normrnd(0, sigma_y0) normrnd(0, sigma_y, 1, step-1)];

V = [normrnd(0, sigma_r, 1, step*k); 
    normrnd(0, sigma_beta, 1, step*k)];

save W W
save V V