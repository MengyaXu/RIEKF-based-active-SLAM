function [I, P_LS, dx, Lg, Lt] = GN(step, k0, u, index_last, X0)
load data_NLS cov_u cov_z Q
% load data_NLS step k u z cov_u cov_z
J_const = [0 -1; 1 0];
XR = [0; 0; 0; X0(4:3*step)];

ex = 0;
ez = 0;
x_est(:, 1) = [0;0;0];
% J = zeros(3*step + 2*k0,3);
J = [];
H = zeros(3*step + 2*k0, 3*step + 2*k0);
H(1,1) = inf;H(2,2) = inf; H(3,3) = inf;
% H(1:3,1:3) = cov_u^(-1);
g = zeros(3*step + 2*k0,1);
Ag = [];
Lg = zeros(step + k0, step + k0);
Lt = zeros(step, step);
for i = 1:step - 1
    Jxi = zeros(3,3*step + 2*k0);
    theta = XR(3*i-2);
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    motion = [u(1, i);rot * u(2:3, i)];
    x_est(:, i+1) = XR(3*i - 2 : 3*i) + motion;
%     ex = ex + (XR(3*i + 1 : 3*i + 3) - x_est(:, i+1))'* cov_u^(-1) *(XR(3*i + 1 : 3*i + 3) - x_est(:, i+1));
%     Jxi(:, 3*i-2:3*i+3) = [-1 0 d*sin(XR(3*i) + alpha) 1 0 0;
%                         0 -1 -d*cos(XR(3*i) + alpha) 0 1 0;
%                         0 0 -1 0 0 1 ];
    Jxi(:, 3*i-2:3*i+3) = [[-1 0 0;
                   -rot * J_const * u(2:3, i), -diag([1 1])] eye(3)];
%     Jxi(:, 3*i-2:3*i+3) = [-1 0 0 1 0 0;
%                         d*sin(XR(3*i - 2) + alpha) -1 0 0 1 0;
%                         -d*cos(XR(3*i - 2) + alpha) 0 -1 0 0 1 ];
    J = [J Jxi'];
    H = H + Jxi' / (cov_u) * Jxi;
    fxi = XR(3*i + 1 : 3*i + 3) - x_est(:, i+1);
                    fxi(1) = rem(fxi(1),2*pi);
                    if rem(fxi(1),2*pi) > pi
                        fxi(1) = 2*pi - rem(fxi(1),2*pi);
                    elseif rem(fxi(1),2*pi) < -pi
                        fxi(1) = 2*pi + rem(fxi(1),2*pi);
                    else
                        fxi(1) = rem(fxi(1),2*pi);
                    end
    
    ex = ex + fxi'/cov_u*fxi;
    gxi = -Jxi' / (cov_u) * fxi;
    g = g + gxi;
    
    Agi = zeros(step + k0, 1);
    Agi(i:i + 1,:) = [-1; 1];
    Ag = [Ag Agi];
    Lg = Lg + Agi * cov_u(2, 2)^(-1) * Agi';
    
    At = zeros(step, 1);
    At(i:i+1) = [-1; 1];
    Lt = Lt + At * cov_u(1)^(-1) * At';
end
    fz = fopen('data/obs.txt','r');
    fid = fopen('data/id.txt', 'r');
    idline = fgetl(fid);
    index = str2num(idline);
%     index_last = index;
    k = length(index);
    XF = X0(3*step + 1:3*step + 2 * k0);
    
for i = 1:step
    z = [];
    zline = fgetl(fz);
    if zline ~= -1
        z = str2num(zline)';
    end
    if length(z) ~= 0
    k = length(index);
    for j = 1:k
        zj = [z(2*j-1); z(2*j)];
%         if ismember(index(j), index_last)
            id = find(index_last == index(j));
%             Jzi = zeros(2,3*step + 2*k0);
            delta_x = XF(2*id - 1) - XR(3*i - 1);
            delta_y = XF(2*id) - XR(3*i);
            delta = [delta_x; delta_y];
            theta = XR(3*i - 2);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            y = rot' * delta;
%             if y(1) < 0.01 || y(1) > 0
%                 y(1) = 0.01;
% %                 di = 0.01
%             elseif  y(1) > -0.01 && y(1) < 0
%                 y(1) = -0.01;
%             end
%             if y(2) < 0.01 && y(2) > 0
%                 y(2) = 0.01;
%             elseif  y(2) > -0.01 && y(2) < 0
%                 y(2) = -0.01;
%             end
            q = y' * y;
            z_est = [sqrt(q); wrapToPi(atan2(y(2),y(1)))];
    %         ez = ez + (z(2*j-1:2*j,i) - z_est)'* cov_z^(-1) *(z(2*j-1:2*j,i) - z_est);
%             Jzi(:, 3*i - 2:3*i) = -(1/q) * [-sqrt(q)*delta_x, -sqrt(q)*delta_y, 0;
%                                   delta_y, -delta_x, -q];
%             Jzi(:, 3*step + 2*id - 1:3*step + 2*id) = -(1/q) * [sqrt(q)*delta_x, sqrt(q)*delta_y;
%                                                     -delta_y, delta_x];
            F_big = zeros(5, (2*k0)+3*step);
            F_big(1:3,3*i-2:3*i) = eye(3);
            F_big(4:5, 3*step+2*id-1:3*step+2*id) = eye(2);

            dh = [y(1)/sqrt(q) y(2)/sqrt(q); -y(2)/q y(1)/q];
            Hij = [-J_const*rot'*delta -rot' rot'];
            Jzi = -dh * Hij * F_big;
            
%             J = J + [Jzi',zeros(3*step + 2*k0,1)];
            J = [J Jzi'];
            H = H + Jzi' / (cov_z) * Jzi;
            fzi = zj - z_est;
                    fzi(2) = rem(fzi(2),2*pi);
                    if rem(fzi(2),2*pi) > pi
                        fzi(2) = 2*pi - rem(fzi(2),2*pi);
                    elseif rem(fzi(2),2*pi) < -pi
                        fzi(2) = 2*pi + rem(fzi(2),2*pi);
                    else
                        fzi(2) = rem(fzi(2),2*pi);
                    end
            ez = ez + fzi'/ cov_z * fzi;
            gzi = -Jzi' / (cov_z) *  fzi;
            g = g + gzi;
            
            Agi = zeros(step + k0, 1);
            Agi(i) = -1;
            Agi(step + id) = 1;
            Ag = [Ag Agi];
            Lg = Lg + Agi * Q(1)^(-1) * Agi';
    end
    end
    idline = fgetl(fid);
    if idline ~= -1
        index = str2num(idline);
    end
end
dx = [zeros(3,1); sparse(H(4:end,4:end))^(-1) * g(4:end)];
I = H;
P_LS = H(4:end,4:end)^(-1);
% Lg = kron(Lg, eys(2));
e = ex + ez;
% -2 * logdet_amd(kron(Lg(2:end, 2:end), eye(2)), 'chol') - logdet_amd(kron(Lt(2:end, 2:end), eye(2)), 'chol')
%  -logdet_amd(I(4:end, 4:end), 'chol')
        fclose(fz);
        fclose(fid);