function [x_next, P_next, u, u0] = initU3(x, P, horizon, u, u0, i)
load SLAM_2D_data k cov_u cov_z mode J
d = [1.5;1.5];
for alpha = 0:pi/8:2*pi
    load Pmin Pmin
    theta = x(3, i);
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        motion = [rot * d;
                    alpha; zeros(2*k,1)];
        x_pre = x(:,i) + motion;
        x_next = x_pre;
%             plot(x_pre(1), x_pre(2), '*r');
        if mode == 1        % EKF
            % Jacobian of predict model
            F_x = [diag([1 1]), rot * J * d;
                    0 0 1];
            F = [F_x zeros(3, 2*k);
                zeros(2*k, 3) eye(2*k)];
            
            G = [rot zeros(2, 2*k+1);
                0 0 1 zeros(1, 2*k);
                zeros(2*k, 2*k+3)];

            R = [cov_u zeros(3,2*k); zeros(2*k,2*k+3)];
            P_pre = F * P * F' + G * R * G';
            P_next = P_pre;
        elseif mode == 3        % IEKF
            F = diag(ones(1, 2*k+3));
            G1 = 1;
            for n = 1:2:2*k-1
                G1 = [G1; -J*x(3+n:3+n+1, i)];
            end
            G = [rot -J*x(1:2, i) zeros(2, 2*k);
                zeros(2*k+1, 1) G1 zeros(2*k+1, 2*k+1)];
            R = [cov_u zeros(3,2*k); zeros(2*k,2*k+3)];
            P_pre = F * P * F' + G * R * G';
            P_next = P_pre;
        end
        for j = 1:k
            if mode == 1 || mode == 3       % update
                delta_x = x_next(3 + 2*j-1) - x_next(1);
                delta_y = x_next(3 + 2*j) - x_next(2);
                delta = [delta_x; delta_y];
                theta = x_next(3);
                rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                y = rot' * delta;
                q = y' * y;
                F_big = zeros(5, (2*k)+3);
                F_big(1:3,1:3) = eye(3);
                F_big(4:5, 3+2*j-1:3+2*j) = eye(2);
                
                dh = [y(1)/sqrt(q) y(2)/sqrt(q); -y(2)/q y(1)/q];
                    Hij = [-rot' -J*rot'*delta rot'];
                    H = dh * Hij * F_big;
                if  mode == 1
                    S = H * P_next * H' + cov_z;
                    K = P_next * H' * inv(S);
                    P_next = P_next - K * S * K';
                elseif mode == 3
                    H(:, 3) = [0;0];
                    S = H * P_next * H' + cov_z;
                    K = P_next * H' * inv(S);
                    P_next = P_next - K * S * K';
                end
            end
        end
    u_next = [u0, [d;theta]];
    
    if i == horizon && trace(P_next) < Pmin
        Pmin = trace(P_next);
        save Pmin Pmin
%         a = alpha;
        u = u_next;
    end
    
    if i < horizon
        [x_next, P_next, u, u_next] = initU3(x_next, P_next, horizon, u, u_next, i+1);
    end
end
u;
end

