function [I, P_LS, dx, Lg, Lt, e] = GN(step, k0, u, index_last, X0)
load data_NLS cov_u cov_z Q
% load data_NLS step k u z cov_u cov_z
J_const = [0 -1; 1 0];
XR = [0; 0; 0; X0(4:3*step)];

ex = 0;
ez = 0;
x_est(:, 1) = [0;0;0];
% J = zeros(3*step + 2*k0,3*step + 2*k0);
% J = [];
% R = zeros(3*step - 3, 3*step - 3);
% f = zeros(3*step + 2*k0, 3);
H = sparse(3*step + 2*k0, 3*step + 2*k0);
H(1,1) = inf;H(2,2) = inf; H(3,3) = inf;
% H(1:3,1:3) = cov_u^(-1);
g = zeros(3*step + 2*k0,1);
Ag = [];
Lg = sparse(step + k0, step + k0);
Lt = sparse(step, step);
for i = 1:step - 1
    Jxi = sparse(3,3*step + 2*k0);
    theta = XR(3*i-2);
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    motion = [u(1, i);rot * u(2:3, i)];
    x_est(:, i+1) = XR(3*i - 2 : 3*i) + motion;
    Jxi(:, 3*i-2:3*i+3) = [[-1 0 0;
           -rot * J_const * u(2:3, i), -diag([1 1])] eye(3)];
    
%     J = [J Jxi'];
%     J(:, 3*i-2:3*i+3) = Jxi';
%                R(3*i-2:3*i, 3*i-2:3*i) = cov_u;
    H = H + Jxi' * inv(sparse(cov_u)) * Jxi;
    fxi = XR(3*i + 1 : 3*i + 3) - x_est(:, i+1);
                    fxi(1) = rem(fxi(1),2*pi);
                    if rem(fxi(1),2*pi) > pi
                        fxi(1) = 2*pi - rem(fxi(1),2*pi);
                    elseif rem(fxi(1),2*pi) < -pi
                        fxi(1) = 2*pi + rem(fxi(1),2*pi);
                    else
                        fxi(1) = rem(fxi(1),2*pi);
                    end
%     f(3*i - 2 : 3*i) = f + fxi;
    ex = ex + fxi'/cov_u*fxi;
    gxi = -Jxi' * inv(cov_u) * fxi;
    g = g + gxi;
    
    Agi = sparse(step + k0, 1);
    Agi(i:i + 1,:) = [-1; 1];
    Ag = [Ag Agi];
    Lg = Lg + Agi * cov_u(2, 2)^(-1) * Agi';
    
    At = sparse(step, 1);
    At(i:i+1) = [-1; 1];
    Lt = Lt + At * cov_u(1)^(-1) * At';
end
    fz = fopen('data/obs.txt','r');
    fid = fopen('data/id.txt', 'r');
%     index_last = index;
    XF = X0(3*step + 1:3*step + 2 * k0);
    
for i = 1:step
    zline = fgetl(fz);
    idline = fgetl(fid);
    if zline ~= -1
        z = str2num(zline)';
        index = str2num(idline)';
        k = length(index);
        for j = 1:k
            zj = [z(2*j-1); z(2*j)];
            id = find(index_last == index(j));
%             Jzi = zeros(2,3*step + 2*k0);
            Jzi = sparse(2,3*step + 2*k0);
            delta_x = XF(2*id - 1) - XR(3*i - 1);
            delta_y = XF(2*id) - XR(3*i);
            delta = [delta_x; delta_y];
            theta = XR(3*i - 2);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            y = rot' * delta;
            q = y' * y;
            z_est = [sqrt(q); wrapToPi(atan2(y(2),y(1)))];

            F_big = sparse(5, (2*k0)+3*step);
            F_big(1:3,3*i-2:3*i) = eye(3);
            F_big(4:5, 3*step+2*id-1:3*step+2*id) = eye(2);

            dh = [y(1)/sqrt(q) y(2)/sqrt(q); -y(2)/q y(1)/q];
            Hij = [-J_const*rot'*delta -rot' rot'];
            Jzi = -sparse(dh) * sparse(Hij) * F_big;

%             J(:, 3 * step + 2 * id - 1:3 * step + 2 * id) = Jzi';
%             J = [J Jzi'];
%             R = [R zeros(length(R), 2);
%                 zeros(2, length(R)) cov_z];
%             R(3 * step + 2 * id - 1:3 * step + 2 * id, 3 * step + 2 * id - 1:3 * step + 2 * id) = cov_z;
            H = H + Jzi' * inv(sparse(cov_z)) * Jzi;
            fzi = zj - z_est;
                    fzi(2) = rem(fzi(2),2*pi);
                    if rem(fzi(2),2*pi) > pi
                        fzi(2) = 2*pi - rem(fzi(2),2*pi);
                    elseif rem(fzi(2),2*pi) < -pi
                        fzi(2) = 2*pi + rem(fzi(2),2*pi);
                    else
                        fzi(2) = rem(fzi(2),2*pi);
                    end
%             f(3 * step + 2 * id - 1:3 * step + 2 * id) = f + fzi;
            ez = ez + fzi'/ cov_z * fzi;
            gzi = -Jzi' * inv(cov_z) *  fzi;
            g = g + gzi;

            Agi = sparse(step + k0, 1);
            Agi(i) = -1;
            Agi(step + id) = 1;
            Ag = [Ag Agi];
            Lg = Lg + Agi * Q(1)^(-1) * Agi';
        end
    end
end
% H = J / R * J';
dx = [zeros(3,1); H(4:end,4:end) \ g(4:end)];
% dx = [zeros(3,1); H(4:end,4:end)^(-1) * g(4:end)];
I = H;
P_LS = [];
% P_LS = inv(H(4:end,4:end));
% Lg = kron(Lg, eys(2));
e = ex + ez;
% -2 * logdet_amd(kron(Lg(2:end, 2:end), eye(2)), 'chol') - logdet_amd(kron(Lt(2:end, 2:end), eye(2)), 'chol')
%  -logdet_amd(I(4:end, 4:end), 'chol')
        fclose(fz);
        fclose(fid);