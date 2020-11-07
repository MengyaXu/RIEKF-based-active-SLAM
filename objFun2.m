function obj = objFun2(u0)
load SLAM_2D_data k mode actual_h x P cov_u cov_z J range
load paraU goal
%     u;
% step = 3;
d = 0;
% op_2 = 2;
for i = 1:1:actual_h
    u(:, i) = [u0(1, i); u0(2, i) * cos(u0(1, i)); u0(2, i) * sin(u0(1, i))];
    theta = x(1);
        rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        motion = [u(1, i); rot * u(2:3, i);
                    zeros(2*k,1)];
        x_pre = x + motion;
        x_next = x_pre;
        
        if length(goal) ~= 0
            delta = x_pre(2:3) - goal;
            d = real(sqrt(delta' * delta));
        end
        
        if mode == 1 %|| mode == 2       % EKF
            % Jacobian of predict model
            F_x = [1 0 0;
                       rot * J * u(2:3, i), diag([1 1])];
                F = [F_x zeros(3, 2*k);
                    zeros(2*k, 3) eye(2*k)];

                G = [1 zeros(1, 2*k+2);
                    zeros(2, 1) rot zeros(2, 2*k);
                    zeros(2*k, 2*k+3)];

                R = [cov_u zeros(3,2*k); zeros(2*k,2*k+3)];
                P_pre = F * P * F' + G * R * G';
                P_next = P_pre;
        elseif mode == 3        % IEKF
            F = diag(ones(1, 2*k+3));
                G1 = 1;
                for n = 1:2:2*k+1
                    G1 = [G1; -J*x(1+n:1+n+1)];
                end
                G = [G1, [zeros(1, 2*k+2); 
                            rot zeros(2, 2*k);
                            zeros(2*k, 2*k+2)]];
                R = [cov_u zeros(3,2*k); zeros(2*k,2*k+3)];
                P_pre = F * P * F' + G * R * G';
                P_next = P_pre;
        elseif mode == 2
            load data_NLS I step_c Lg Lt Q op_2
            if op_2 == 1
                % ==========
                I_new = [I(1:3*step_c, 1:3*step_c) zeros(3*step_c,3) I(1:3*step_c, 3*step_c+1:end);
                    zeros(3, 3*step_c+3+2*k);
                    I(3*step_c+1:end, 1:3*step_c) zeros(2*k, 3) I(3*step_c+1:end, 3*step_c+1:end)];
                I_new = sparse(I_new);
%                 I_new = sparse(3 * step_c + 3 + 2 * k, 3 * step_c + 3 + 2 * k);
%                 I_new(1:3*step_c, 1:3*step_c) = I(1:3*step_c, 1:3*step_c);
                
                
                Jxi = sparse(3,3*step_c+3 + 2*k);
                Jxi(:, 3*step_c-2:3*step_c+3) = [[-1 0 0;
                       -rot * J * u(2:3, i), -diag([1 1])] eye(3)];
%                    Jxi' / (cov_u) * Jxi;
                I_new = I_new + Jxi' / sparse(cov_u) * Jxi;
                % =============
            else
                Lg;
                Lg = [Lg(1:step_c+i-1, 1:step_c+i-1) zeros(step_c+i-1, 1) Lg(1:step_c+i-1, step_c+i:end);
                    zeros(1, step_c+i + k);
                    Lg(step_c + i:end, 1:step_c+i-1) zeros(k, 1) Lg(step_c+i:end, step_c+i:end)];
%                 a = [1:step_c+i-1, 1:step_c+i-1, step_c + i:step_c + i + k, step_c + i:step + i +k];
%                 b = [1:step_c+i-1, step_c + i:step + i +k,1:step_c+i-1 , step_c + i:step + i +k];
%                 c = 
                Agi = sparse(step_c + i + k, 1);
                Agi(step_c + i - 1: step_c + i) = [-1; 1];
                %     Ag = [Ag Agi];
                Lg = Lg + Agi * cov_u(2, 2)^(-1) * Agi';
                
                Lt = sparse([Lt zeros(step_c + i - 1, 1);
                    zeros(1, step_c + i)]);
                At = zeros(step_c + i, 1);
                At(step_c + i - 1: step_c + i) = [-1; 1];
                Lt = Lt + At / cov_u(1) * At';
            end
        end
        
        % update
        for j = 1:k
            delta_x = x_next(3 + 2*j-1) - x_next(2);
            delta_y = x_next(3 + 2*j) - x_next(3);
            delta = [delta_x; delta_y];
            theta = x_next(1);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            y(:, j) = rot' * delta;
%             if y(1) < 0.001 || y(1) > 0
%                 y(1) = 0.001;
%             elseif  y(1) > -0.001 && y(1) < 0
%                 y(1) = -0.001;
%             end
%             if y(2) < 0.001 && y(2) > 0
%                 y(2) = 0.001;
%             elseif  y(2) > -0.001 && y(2) < 0
%                 y(2) = -0.001;
%             end
            q = y(:, j)' * y(:, j);
            dd = real(sqrt(q));
            if dd <= range && dd >= 0.1
            if mode == 1 || mode == 3 %|| mode == 2
                F_big = zeros(5, (2*k)+3);
                F_big(1:3,1:3) = eye(3);
                F_big(4:5, 3+2*j-1:3+2*j) = eye(2);

                dh = [y(1,j)/sqrt(q) y(2,j)/sqrt(q); -y(2,j)/q y(1,j)/q];
                Hij = [-J*rot'*delta, -rot', rot'];
                H = dh * Hij * F_big;
                if  mode == 1 %|| mode == 2
                    S = H * P_next * H' + cov_z;
                    K = P_next * H' / (S);
                    P_next = P_next - K * S * K';
                elseif mode == 3
                    H(:, 1) = [0;0];
                    S = H * P_next * H' + cov_z;
                    K = P_next * H' / (S);
                    P_next = P_next - K * S * K';
                end
            elseif mode == 2
                if op_2 == 1
                    F_big = zeros(5, (2*k)+3*step_c+3);
                    F_big(1:3,3*step_c+1:3*step_c+3) = eye(3);
                    F_big(4:5, 3*step_c+3+2*j-1:3*step_c+3+2*j) = eye(2);

                    dh = [y(1,j)/sqrt(q) y(2,j)/sqrt(q); -y(2,j)/q y(1,j)/q];
                    Hij = [-J*rot'*delta, -rot', rot'];
                    Jzi = -dh * Hij * F_big;
                    I_new = I_new + Jzi' / (cov_z) * Jzi;
                    % =========
                else
                    Agi = sparse(step_c + i + k, 1);
                    Agi(step_c + i) = -1;
                    Agi(step_c + i + j, :) = 1;
                    Lg = Lg + Agi * Q(1)^(-1) * Agi';
                end
            end
            end
        end
            x = x_next;
        if mode == 1 || mode == 3 %|| mode == 2
            P = (P_next + P_next')/2;
        elseif mode == 2
            if op_2 == 1
                I = (I_new + I_new')/2;
                y;
            else
                Lg = Lg;
            end
        end
end
if mode ==1 || mode == 3 %|| mode == 2
    obj1 = trace(P_next);
        t= log10(obj1);
        scale = floor(t);
        obj = obj1 + 10^scale * d * 0.2;
%     obj = logdet_amd(P) + 10*d;
elseif mode == 2
    if op_2 == 1
        obj1 = -logdet_amd(I(4:end, 4:end), 'chol');
        t= log10(obj1);
        scale = real(floor(t));
        obj = -obj1 + 10^scale * d * 0.1;
    else
        Lg = Lg(2:end, 2:end);
        Lg = kron(Lg, eye(2));
        Lp = kron(Lt(2:end, 2:end), eye(2));
        obj1 = 2 * logdet_amd(Lg, 'chol') + logdet_amd(Lp, 'chol');
        t= log10(obj1);
        scale = real(floor(t));
        obj = -obj1 + 10^scale * d * 0.08;
    end
end
end

