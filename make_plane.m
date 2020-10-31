function fig = make_plane(p, alpha, scalar, color)
% plot a plane
% alpha means orientation, scalar means the size, p is coordinate,
% p=[0,0];
% alpha=3*pi;
% scalar=1;
% color=[1,0,0];

[n, m] = size(p);
if m == 2         
    p = p';
end

alpha = alpha - pi/2;
R = [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];

% colour

t = [0, pi/2, pi, 2*pi]';
x = scalar * cos(t) .* [2, 1, 2, 2]';
y = scalar * sin(t) .* [1, 2, 1, 1]';
for i = 1:4
    mid = R * [x(i); y(i)];
    x(i) = mid(1);
    y(i) = mid(2);
end
x = p(1) + x;
y = p(2) + y;
fill(x,y, color);
hold on;

t = [0, pi/2, pi, 2*pi]';
x = scalar * cos(t);
y = scalar * sin(t) .* [1, 5, 1, 1]';
for i = 1:4      % rotation
    mid = R * [x(i); y(i)];
    x(i) = mid(1);
    y(i) = mid(2);
end
x = p(1) + x;     % translation
y = p(2) + y;
fig = fill(x,y,color);

axis equal;