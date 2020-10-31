clear;
close all;
clc;
%%

    round = 1;
for e = 1:round
sigma_x = 0.1;
sigma_y = 0.1;
sigma_theta = 0.1;
sigma_r = 0.04;
sigma_beta = 0.04;

cov_init = diag([0.001, 0.001, 0.001]);
cov_u = diag([sigma_theta^2, sigma_x^2, sigma_y^2]);
cov_z = diag([sigma_r^2, sigma_beta^2]);
save data_NLS cov_u cov_z

step = 300;      % number of total steps
x0 = [0; 0; 0];   % initial pose
u = [];
range = 20;
run = 1;
neesr = 0;
head = 0;

op_2 = 1;
resultId = 5;
mode = 4;   % 1 is EKF, 3 is IEKF
op = 1;    % for active SLAM, ignore it here

w = 10;
range_goal = 40;

load map_obs L0 obstacle

Le0 = [0 0 0 100 100 100 100 0];
Le = Le0;
k0 = length(L0)/2;
count = 0;

J = [0 -1; 1 0];
save SLAM_2D_data k0 step cov_u cov_z J obstacle range

% ===== Generate initial observation data =====
z = [];
L = [];
index_last = [];
num_id = 20;
index_recent = [];
pose_act = x0;  % actual robot pose

if mode ~= 4
    flag = 1;
else
    flag = 0;
end
goal = [0;0];

sl = 0;     % generate observation data
if sl == 0
    fr = fopen('data/pose.txt','w+');
%     fx = fopen('x_noise_in_pre3.txt', 'w+');
    fz = fopen('data/obs.txt','w+');
%     fz0 = fopen('obs_no_noise.txt','w+');
    fid = fopen('data/id.txt', 'w+');
    fprintf(fr, '%f\t%f\t%f\n', pose_act(1),pose_act(2),pose_act(3));
end

sl2 = 1;
if sl2 == 1
    if mode == 1
        str = 'EKF';
    elseif mode == 3
        str = 'IEKF';
    elseif mode == 2 && op_2 == 1
        str = 'NLSI';
    elseif mode == 2 && op_2 == 2
        str = 'NLSlb';
    elseif mode == 4
        str = 'NLSall';
    end
    if op == 1
        str_op = 'pre_';
    elseif op == 4
        str_op = '';
    end
    fer = fopen(['result/rand_' str_op str '_er_' num2str(resultId) '.txt'], 'w+');
    fef = fopen(['result/rand_' str_op str '_ef_' num2str(resultId) '.txt'], 'w+');
    ftp = fopen(['result/rand_' str_op str '_tp_' num2str(resultId) '.txt'], 'w+');
    ftu = fopen(['result/rand_' str_op str '_tu_' num2str(resultId) '.txt'], 'w+');
    
    fp = fopen(['result/rand_' str_op str '_p_' num2str(resultId) '.txt'], 'w+');
    fxa = fopen(['result/rand_' str_op str '_xa_' num2str(resultId) '.txt'], 'w+');
    fxr = fopen(['result/rand_' str_op str '_xr_' num2str(resultId) '.txt'], 'w+');
    fxf = fopen(['result/rand_' str_op str '_xf_' num2str(resultId) '.txt'], 'w+');
    fxls = fopen(['result/rand_' str_op str '_xls_' num2str(resultId) '.txt'], 'w+');
    
    fprintf(fxa, '%f\t%f\t%f\n', pose_act(1),pose_act(2),pose_act(3));
    fprintf(fxr, '%f\t%f\t%f\n', x0(1),x0(2),x0(3));
end
    
    for j = 1:k0
        delta_x = L0(2*j-1) - x0(2);
        delta_y = L0(2*j) - x0(3);
        delta = [delta_x; delta_y];
        theta = x0(1);
        rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        y = rot' * delta;
        q = y' * y;
        if sqrt(q) <= range
            V = [normrnd(0, sigma_r); normrnd(0, sigma_beta)];
%             V = [0;0];
            zj = [sqrt(q);
                wrapToPi(atan2(y(2),y(1)))] + V;
            z = [z; zj];
            index_last = [index_last; j];
            L = [L L0(2*j-1:2*j)];
            if sl == 0
                fprintf(fz, '%f\t', zj);
                fprintf(fid, '%f\t', j);
            end
%             plot(L0(2*j-1),L0(2*j),'ob');
%             hold on;
        end
%         size(z)
    end
if sl == 0
    fprintf(fz, '\n');
    fprintf(fid, '\n');
end
k_init = length(index_last);
k = k_init;
% ===== Calculate initial landmark vector =====
landmark = zeros(2*k,1);
cov_landmark = zeros(2*k);

Lg = zeros(1+k, 1+k);
Lt = 0;
for i = 1:2:2*k
   Cos = cos(x0(1) + z(i+1));
   Sin = sin(x0(1) + z(i+1));
   % Jacobian of landmark
   jacobian_landmark = [Cos -z(i)*Sin;
                   Sin z(i)*Cos];
   jr = [-z(i)*Sin 1 0;
       z(i)*Cos 0 1];
   if mode == 3
       jr(:,1) = [0;0];
   end
   Q = jacobian_landmark * cov_z * jacobian_landmark' + jr * cov_init * jr';
   cov_landmark(i:i+1,i:i+1) = Q;
   
    x_lm = x0(2) + z(i) * cos(x0(1) + z(i+1));
    y_lm = x0(3) + z(i) * sin(x0(1) + z(i+1));
    landmark(i) = x_lm;
    landmark(i+1) = y_lm;

    Agi = zeros(1 + k, 1);
    Agi(1) = -1;
    Agi(1 + (i+1)/2) = 1;
    Lg = Lg + Agi * Q(1)^(-1) * Agi';
end

er_EKF = 0;     % weighted error of robot pose
er2 = 0;
ef_EKF = 0;     % weighted error of landmark
    
%%
    k = k_init;
%     W = [0 0 0]';
    W = [normrnd(0, sigma_theta^2); normrnd(0, sigma_x^2); normrnd(0, sigma_y^2)];
    x = [x0+W; landmark];
%     P = [zeros(3,3) zeros(3, 2*k);
%         zeros(2*k, 3) cov_landmark];
    P = [cov_init zeros(3, 2*k);
        zeros(2*k, 3) cov_landmark];
                            
     P_last = P;
%     radius = sqrt(P(1,1));
%     fprintf(fx, '%f\t%f\t%f\n', x(1),x(2),x(3));

    if mode == 2 || mode == 4
        X_LS = x;
        xr = x(1:3);
        xf = x(4:end);
        step_c = 1;
        I = P^(-1);
%         I = [diag([inf inf inf]) zeros(3, 2*k);
%         zeros(2*k, 3) diag(ones(1, 2*k)*cov_z(1))^(-1)];
%         I
        g =  zeros(3 + 2*k,1);
        save data_NLS I g Lg Lt Q step_c op_2 '-append'
    end
    % ---------------
    figure(mode)
    for i = 1:k0
        h_gl = plot(L0(2*i-1), L0(2*i), '+black','MarkerSize',10);
        hold on;
    end
    for j = 1:length(obstacle)/3
        r = obstacle(3*j);
        c = obstacle(3*j - 2:3*j - 1);
        phi = [0:(pi/50):(2*pi)];
        for i = 1:101
            rect = [r*cos(phi(i)) ; r*sin(phi(i))];
            obs(:,i) = rect + c;
        end
        plot(obs(1,:), obs(2,:), '-black');
        hold on;
    end
    
    make_plane(x(2:3), x(1), 0.75, 'b');
    % ---------------
    ku = k;
    save paraU flag goal w range_goal ku
    b = 0;
    % =========== main ==============
    for i = 1:step-1
%         pause(0.5);
        save SLAM_2D_data mode x P '-append'
        index = [];
        j_delet = 0;
%         if length(Le) == 0
%             Le = [0 0];
%         end
        for j = 1:length(Le)/2
            delta = x(2:3) - Le(2*j - 1:2*j)';
            d = real(sqrt(delta' * delta));
            if d < range 
                j_delet = j;
            end
        end
        if j_delet ~= 0
            Le(2*j_delet - 1:2*j_delet) = [];
        end
        
        tic
        u = getU(op, step, range, Le, k, i, x, P_last, P, u);
        toc;
        tp = toc;
        
            % ----- generate ground truth -----
            % W = [normrnd(0, sigma_theta); normrnd(0, sigma_x); normrnd(0, sigma_y)];
            W = [0; 0; 0];
            theta = pose_act(1, i);
            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            motion_actual = [u(1, i) + W(1);
                            rot * (u(2:3, i) + W(2:3))];
            pose_act(:, i+1) = pose_act(:,i) + motion_actual;
            % -------------------------------
            for r = 1:run
        % ===== Predict =====
                W = [normrnd(0, sigma_theta^2); normrnd(0, sigma_x^2); normrnd(0, sigma_y^2)];
%                 W = [0; 0; 0];
                theta = x(1);
                rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                motion = [u(1, i) + W(1);rot * (u(2:3, i) + W(2:3));
                             zeros(2*k,1)];
                x_pre = x + motion;
                x_next = x_pre;
%                 plot(x_pre(2), x_pre(3), '*m');

            if mode == 1        % EKF
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
            end
            z = [];
            tic
            for j = 1:k0
                % ----- generate observation ----
                delta_x = L0(2*j - 1) - pose_act(2, i+1);
                delta_y = L0(2*j) - pose_act(3, i+1);
                delta = [delta_x; delta_y];
                theta = pose_act(1, i+1);
                rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                y = rot' * delta;
                q = y' * y;
                V = [normrnd(0, sigma_r^2); normrnd(0, sigma_beta^2)];
%                 V = [0; 0];
                zj = [sqrt(q);
                    wrapToPi(atan2(y(2),y(1)))] + V;
                if zj(1) <= range && zj(1) >= 0.1
                    z = [z; zj];
                    if sl == 0
                        fprintf(fz, '%f\t', zj);
                        %fprintf(fz0, '%f\t', zj0);
                        fprintf(fid, '%f\t', j);
                    end
%                     index_recent = [index_recent; j];
%                     if length(index_recent) > num_id
%                         index_recent = index_recent(2:end);
%                     end
                    if ismember(j, index_last)
                        if mode == 1 || mode == 3
                            id = find(index_last == j);
                            delta_x = x_next(3 + 2*id-1) - x_next(2);
                            delta_y = x_next(3 + 2*id) - x_next(3);
                            delta = [delta_x; delta_y];
                            theta = x_next(1);
                            rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
                            y = rot' * delta;
                            q = y' * y;
                            z_est = [sqrt(q);
                                wrapToPi(atan2(y(2),y(1)))];

                            dz = zj - z_est;
                            dz(2) = rem(dz(2),2*pi);
                            if rem(dz(2),2*pi) > pi
                                dz(2) = 2*pi - rem(dz(2),2*pi);
                            elseif rem(dz(2),2*pi) < -pi
                                dz(2) = 2*pi + rem(dz(2),2*pi);
                            else
                                dz(2) = rem(dz(2),2*pi);
                            end
    %                         dz(2) = min(abs(rem(2*pi-rem(dz(2),2*pi),2*pi)), abs(rem(2*pi+rem(dz(2),2*pi),2*pi)));
    %                         if abs(rem(2*pi-rem(dz(2),2*pi),2*pi)) > abs(rem(2*pi+rem(dz(2),2*pi),2*pi))
    %                             dz(2) = rem(2*pi+rem(dz(2),2*pi),2*pi);
    %                         else
    %                             dz(2) = rem(2*pi-rem(dz(2),2*pi),2*pi);
    %                         end
                            F_big = zeros(5, (2*k)+3);
                            F_big(1:3,1:3) = eye(3);
                            F_big(4:5, 3+2*id-1:3+2*id) = eye(2);

                            dh = [y(1)/sqrt(q) y(2)/sqrt(q); -y(2)/q y(1)/q];
                                Hij = [-J*rot'*delta, -rot', rot'];
                                H = dh * Hij * F_big;
                            if  mode == 1
                                S = H * P_next * H' + cov_z;
                                K = P_next * H' /(S);
                                P_next = P_next - K * S * K';
                                x_next = x_next + K * dz;
                            elseif mode == 3
                                H(:, 1) = [0;0];
                                S = H * P_next * H' + cov_z;
                                K = P_next * H' /(S);
                                P_next = P_next - K * S * K';

                                dx = K * dz;
                                dtheta = dx(1);
                                rot = [cos(dtheta), -sin(dtheta); sin(dtheta), cos(dtheta)];
                                B = [sin(dtheta), cos(dtheta)-1; 1-cos(dtheta), sin(dtheta)]/dtheta;
                                xf3 = x_next(1)+dtheta;
                                for n = 1:2:2*k+1
                                    xf3 = [xf3; rot * x_next(1+n:2+n) + B * dx(1+n:2+n)];
                                end
                                x_next = xf3;
                            end
                        end
                    % ======== new landmark=========
                    else
                        if r == run
                            if mode == 1 || mode == 3
                                xlj = [x_next(2)+zj(1)*cos(x_next(1)+zj(2)); x_next(3)+zj(1)*sin(x_next(1)+zj(2))];
                                x_next = [x_next; xlj];
                                    Hr = [-zj(1)*sin(x_next(1)+zj(2)) 1 0; zj(1)*cos(x_next(1)+zj(2)) 0 1];
                                    Hv = [cos(x_next(1)+zj(2)), -zj(1)*sin(x_next(1)+zj(2));
                                        sin(x_next(1)+zj(2)), zj(1)*cos(x_next(1)+zj(2))];
                                if mode == 3
                                    Hr(:, 1) = [0; 0];
                                end

                                Pf = Hr*P_next(1:3,1:3)*Hr' + Hv*cov_z*Hv';
                                Prf = Hr * P_next(1:3, :);
                                P_next = [P_next Prf';Prf Pf];

    %                             cov_landmark = [cov_landmark zeros(2*k, 2);
    %                                 zeros(2, 2*k) Q];

                            elseif mode == 2 || mode == 4
                                xlj = [x_next(2)+zj(1)*cos(x_next(1)+zj(2)); x_next(3)+zj(1)*sin(x_next(1)+zj(2))];                    
                                X_LS = [X_LS; xlj];
                                xf = [xf; xlj];
                            end
                            index_last = [index_last; j];
                            index = [index; j];
                            k = length(index_last);
                            L = [L L0(2*j-1:2*j)];
                            if length(L) == length(L0) && count == 0
                                count = i;
                            end
                        end
                    end
                    % ==================
%                     h_el = drawCovEllipse(x_next((3 + id*2 - 1):(3 + id*2)), P_next((3 + id*2 - 1):(3 + id*2), (3 + id*2 - 1):(3 + id*2)), 'g');
                end
            end
            
            if mode == 2 || mode == 4
                xr = [xr; x_pre(1:3)];
                if mode == 2
                    if b == 0
                        [X_LS, P_LS, I, b, Lg, Lt] = SLAM_NLS(i+1, k, u, index_last, X_LS, 2);
                    else
                        [X_LS, P_LS, I, b, Lg, Lt] = SLAM_NLS(i+1, k, u, index_last, [xr; xf], 1);
                    end
                    X_LS;
                    step_c = i+1;
                    Lg1 = Lg;
                    save data_NLS I g Lg Lt step_c '-append'
                    x_last = x;
                    x = X_LS(3*i+1:end);
                    P_last = P;
                    P = P_LS(3*i-2:end, 3*i-2:end);
    %                 P = P_LS(3*i+1:end, 3*i+1:end);
                elseif mode == 4
                    x = [x_pre(1:3); xf];
                    if i == step - 1
                        [X_LS, P_LS, I, b, Lg, Lt] = SLAM_NLS(i+1, k, u, index_last, [xr; xf], 1);
                        x_last = x;
                        x = X_LS(3*i+1:end);
                        P_last = P;
                        P = P_LS(3*i-2:end, 3*i-2:end);
                    end
                end
            end
            toc;
            tu = toc;
            
            
            if sl == 0
%                 fprintf(fr, '%f\t%f\t%f\n', pose_act(1,i+1),pose_act(2,i+1),pose_act(3,i+1)');
                fprintf(fr, '%f\t%f\t%f\n', x_pre(1),x_pre(2),x_pre(3)');
                fprintf(fz, '\n');
%                 fprintf(fz0, '\n');
                fprintf(fid, '\n');
            end
            if sl2 == 1
                fprintf(fxa, '%f\t%f\t%f\n', pose_act(1, i+1),pose_act(2, i+1),pose_act(3, i+1));
                fprintf(fxr, '%f\t%f\t%f\n', x_next(1),x_next(2),x_next(3));
            end
            % =================
            if neesr == 1
                dth1 = min(abs(rem(2*pi-rem(x_next(1)-pose_act(1,i+1),2*pi),2*pi)), abs(rem(2*pi+rem(x_next(1)-pose_act(1,i+1),2*pi),2*pi)));
                er = [dth1; x_next(2:3)-pose_act(2:3,i+1)];
                if mode == 3
                    er(2) = er(2) + dth1*x_next(3);
                    er(3) = er(3) - dth1*x_next(2);
                end
%                 er = pose_act(1:3, i+1) - x_next(1:3);
                NEESr(r) = er' * P_next(1:3, 1:3)^(-1) * er;
            end
            end

%     fprintf(fx, '%f\t%f\t%f\n', x(1),x(2),x(3));
        if mode == 1 || mode == 3
            x_last = x;
            x = x_next;
            P_last = P;
            P = P_next;
        end
%         P = (P_next + P_next')/2;

        % ------- error calculation ---------
        if mode == 1 || mode == 3
        dth(i+1, e) = min(abs(rem(2*pi-rem(x(1)-pose_act(1,i+1),2*pi),2*pi)), abs(rem(2*pi+rem(x(1)-pose_act(1,i+1),2*pi),2*pi)));
        er = [dth(i+1, e); x(2:3)-pose_act(2:3,i+1)];
        ef = x(4:end) - L';
%         er_EKF(i) = er' * cov_u^(-1) * er;
        er_EKF(i) = er' * P(1:3, 1:3)^(-1) * er;
        
        er2(i) = er' * er;
        if sl2 == 1
        fprintf(fer, '%f', er2(i));
        fprintf(fer, '\n');
        end
        
        if head == 1
            radius(i+1, e) = sqrt(P(1,1));
            figure(9)
            plot([i, i+1], [sum(radius(i, :)) sum(radius(i+1, :))]/e, '-r');
            hold on;
            plot([i, i+1], -[sum(radius(i, :)) sum(radius(i+1, :))]/e, '-r');
            hold on;
            plot([i, i+1], [sum(dth(i, :)) sum(dth(i+1, :))]/e, '-b');
            hold on;
        end

        if neesr == 1
            NEESrs(i, e) = sum(NEESr)/run/3;
            if i >= 2
                figure(9)
                hold on;
                plot([i - 1, i], [sum(NEESrs(i-1, :))/e, sum(NEESrs(i, :))/e], '-b');
                hold on;
            end
%             NEESr_last(e) = sum(NEESr)/run/3;
        end
        end
        % --------------------

        if mode == 1 || mode == 3
            setting_r = '--b';
            setting_f = 'xr';
            setting_ur = 'b';
            setting_uf = 'r';
        else
            setting_r = '--g';
            setting_f = 'xr';
            setting_ur = 'b';
            setting_uf = 'r';
        end

        % ------ plot EKF ---------
        figure(mode)
%         h_gr = plot([pose_act(2,i) pose_act(2,i+1)], [pose_act(3,i) pose_act(3,i+1)], 'x-black', 'MarkerSize',10);
        h_gr = plot([pose_act(2,i) pose_act(2,i+1)], [pose_act(3,i) pose_act(3,i+1)], '-black', 'MarkerSize',20);
        if mode == 1 || mode == 3
%             h_pr = drawCovEllipse(x_pre(2:3), P_pre(2:3, 2:3), 'm');
%             hold on;
%             h_r = plot([x_last(2) x(2)], [x_last(3) x(3)], setting_r, 'MarkerSize',10);
            if mod(i, 2) == 0
                make_plane(x(2:3), x(1), 0.75, 'b');
                hold on;
            end
%             h_ur = drawCovEllipse(x(2:3), P(2:3, 2:3), setting_ur);
%             hold on;
        elseif mode == 2
            h_r = plot([x_last(2) x(2)], [x_last(3) x(3)], setting_r, 'MarkerSize',10);
        end
        % ---------------
        if sl2 == 1
        fprintf(ftp, '%f', tp);
        fprintf(ftp, '\n');
        fprintf(ftu, '%f', tu);
        fprintf(ftu, '\n');
        end
        if x(2) - pose_act(2, i+1) > 10
            pause(2);
        end
    end
%%
    if sl == 0
        fclose(fr);
        fclose(fz);
        fclose(fid);
%         fclose(fz0);
%         fclose(fx);
    end

    %%
    % ----- plot NLS && e -----
    figure(mode)
    if mode == 2 || mode == 4
        for i = 1:step
            er = X_LS(3*i-2: 3*i) - xr(3*i-2: 3*i);
            er2(i) = er' * er;
            if sl2 == 1
            fprintf(fer, '%f', er2(i));
            fprintf(fer, '\n');
            end
            if mod(i, 3) == 0
                make_plane(X_LS(3*i-1:3*i), X_LS(3*i-2), 0.75, 'b');
            end
            if sl2 == 1
                fprintf(fxls, '%f\t%f\t%f\n', X_LS(3*i-2), X_LS(3*i-1), X_LS(3*i));
            end
        end
        hold on;
    end
    
%     if mode == 1 || mode == 3 || mode == 2 || mode == 4
% %         ef_EKF = ef_EKF + (x(4:end)-L')'*cov_landmark^(-1)*(x(4:end)-L');
%         ef_EKF = ef_EKF + (x(4:end)-L')'*P(4:end,4:end)^(-1)*(x(4:end)-L');
%         ef2 = ef_EKF + (x(4:end)-L')' * (x(4:end)-L');
%     end
    % ------- plot landmark -------
    figure(mode)
    for i = 1:k
        if mode == 1 || mode == 2 || mode == 4
            h_el = GetCov(P((3 + i*2 - 1):(3 + i*2), (3 + i*2 - 1):(3 + i*2)), x(3 + i*2 - 1), x(3 + i*2));
        else
            P_cov = [P(1, 1) zeros(1, 2);
                zeros(2, 1) P((3 + i*2 - 1):(3 + i*2), (3 + i*2 - 1):(3 + i*2))];
            h_el = IEKF_GetCov(P_cov, x(3 + i*2 - 1), x(3 + i*2));
        end
        hold on;
        h_l = plot(x(3 + 2*i -1), x(3+ 2*i), setting_f,'MarkerSize',10);
        hold on;
        
        ef = x((3 + i*2 - 1):(3 + i*2)) - L((i*2 - 1):(i*2))';
        ef2(i) = ef' * ef;
        id = index_last(i);
        if sl2 == 1
        fprintf(fef, '%d\t%f', id, ef2(i));
        fprintf(fef, '\n');
        fprintf(fxf, '%f\t%f\n', x(3 + 2*i -1), x(3+ 2*i));
        for j = 1:k
            fprintf(fp, '%f\t%f\t', P(3 + i*2 - 1,3 + j*2 - 1), P(3 + i*2 - 1,3 + j*2));
        end
            fprintf(fp, '\n');
        for j = 1:k
            fprintf(fp, '%f\t%f\t', P(3 + i*2,3 + j*2 - 1), P(3 + i*2,3 + j*2));
        end
            fprintf(fp, '\n');
        end
    end
    % ------ legend ------
%     fig = [h_gl h_gr h_ur h_r h_el h_l];
    gl = 'Landmark (L)';
    gr = 'Actual robot pose (xa)';
%     pr = 'Uncertainty of the robot pose after prediction';
    ur = 'Uncertainty of the robot pose after update';
    r = 'Predicted robot pose (xr)';
    el = 'Uncertainty of the landmark';
    l = 'Estimate of the landmark (xf)';
%     [leg_EKF, obg, p, str] = legend(fig,gl,gr,ur,r,el,l);
%     axis([-2 16 -1 16]);
%     leg_NLS = copyobj([leg_EKF, gca], gcf);
end
    e_EKF = sum(er_EKF) + sum(ef_EKF);
    ef_total = sum(ef2);
    er_total = sum(er2);
    trace_P = trace(P);
    num = (length(L0) - length(L))/2;
    %NEES;
if e ~= round
    close all;
end
% end
er_total
ef_total
e_EKF
trace_P
num
count
if sl2 == 1
        fclose(fer);
        fclose(fef);
        fclose(ftp);
        fclose(ftu);
        fclose(fp);
        fclose(fxa);
        fclose(fxr);
        fclose(fxf);
        fclose(fxls);
end