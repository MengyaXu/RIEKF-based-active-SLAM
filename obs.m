function [f1, f2] = obs(u0)
load SLAM_2D_data x obstacle
u = [u0(1); u0(2) * cos(u0(1)); u0(2) * sin(u0(1))];
if length(obstacle) ~= 0
theta = x(1);
rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
motion = rot * u(2:3);
xr = x(2:3) + motion;

for n = 1:length(obstacle)/3
    delta = xr - obstacle(3*n-2:3*n-1);
    d(n) = real(sqrt(delta' * delta)) - obstacle(3*n) - 1;
end
f1 = -d;
f2 = [];
else
    f1 = [];
    f2 = [];
end
end

