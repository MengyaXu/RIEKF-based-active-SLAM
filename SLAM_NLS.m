function [X_LS, P_LS, I, b, Lg, Lt] = SLAM_NLS(step, k0, u, index_last, X_last, op_init)
%     load data_NLS k step u z cov_u cov_z
% op_init = 1;
b = 0;
if op_init == 1
    X0 = X_last;
    X0 = X0 + 1 * rand(3*step+2*k0,1);
%     X0 = [0; 0; 2];
%     for i = 1:step-1
%         theta = X0(1);
%         rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%         motion = [u(1, i);rot * u(2:3, i)];
%         X0(3*i+1:3*i+3) = X0(3*i - 2 : 3*i) + motion;
%     end
%     X0 = [X0; zeros(2*k0, 1)];
    
%     X0 = zeros(3*step + 2*k0, 1);
    
%     for j = 1:2:2*k0
%         x_lm = X0(2) + z(j+1,1) * cos(X0(3) + z(j,1));
%         y_lm = X0(3) + z(j+1,1) * sin(X0(3) + z(j,1));
%         X0(3*step + j:3*step + j + 1) = [x_lm; y_lm];
%     end
elseif op_init == 2

%     fid = fopen('id_noise_in_pre.txt', 'r');
%     idline = fgetl(fid);
%     index = str2num(idline);
%     k = length(index);
    X0 = zeros(3*step+2*k0,1);
    i = step - 1;
    X0(1:3*i) = X_last(1:3*i);
    theta = X0(1);
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    motion = [u(1, i);rot * u(2:3, i)];
    X0(3*i+1:3*i+3) = X0(3*i - 2 : 3*i) + motion;
    X0(3*step+1:end) = X_last(3*i+1:end);
elseif op_init == 3
    X0 = zeros(3 * step + 2 * k0, 1);
elseif op_init == 4
    load X_LS X_LS
    X0 = X_LS;
end
    X0;
    %%
            c = 10^(-6);
            [I, P_LS,dx, Lg, Lt, e] = GN(step, k0, u, index_last, X0);
            ep = 0;
            while sum(dx) > c || sum(dx) < -c
                X0 = X0 + dx;
                for i = 1: step
                    X0(3*step - 2) = rem(X0(3*step - 2),2*pi);
                    if rem(X0(3*step - 2),2*pi) > pi
                        X0(3*step - 2) = rem(X0(3*step - 2),2*pi) - 2*pi;
                    elseif rem(X0(3*step - 2),2*pi) < -pi
                        X0(3*step - 2) = 2*pi + rem(X0(3*step - 2),2*pi);
                    else
                        X0(3*step - 2) = rem(X0(3*step - 2),2*pi);
                    end
                end
                [I, P_LS, dx, Lg, Lt, e] = GN(step, k0, u, index_last, X0);
                ep = ep+1;
%                 ep
%                 e
                if ep > 30
                    b = 1
                    dx;
                    break;
                end
            end
            
            P_LS = full(I(4:end, 4:end)^(-1));
            
%             ep
            X_LS = X0;
            setting_r = '.--m';
            setting_f = '.g';
%         hr_NLS = plot(X_LS(1:3:3*step-2), X_LS(2:3:3*step-1), setting_r,'MarkerSize',10);
%         hold on;
%         hl_NLS = plot(X_LS(3*step+1:2:end-1), X_LS(3*step+2:2:end), setting_f,'MarkerSize',5);
%         hold on;
    %%
    trace(P_LS);
    P_LS;
    %%
%     gl = 'Landmark';
%     gr = 'Actual robot pose';
%     % pr = 'Uncertainty of the robot pose after prediction';
%     ur = 'Uncertainty of the robot pose';
%     r = 'Estimate of the robot pose';
%     el = 'Uncertainty of the landmark';
%     l = 'Estimate of the landmark';
%     r_NLS = 'Robot pose estimated using NLS';
%     l_NLS = 'Landmark estimated using NLS';
%     ur_NLS = 'Uncertainty of the robot pose using NLS';
%     ul_NLS = 'Uncertainty of the landmark using NLS';
%     
%     if op == 2
%         for i = 1:step-1
%             hur_NLS = drawCovEllipse(X_LS(3*i+1:3*i+2), P_LS(3*i-2:3*i-1,3*i-2:3*i-1), 'm');
%             hold on;
%         end
%         for j = 1:k
%             hul_NLS = drawCovEllipse(X_LS(3*step + 2*j - 1:3*step + 2*j), P_LS(3*step + 2*j - 4:3*step + 2*j-3, 3*step + 2*j - 4:3*step + 2*j-3), 'g');
%             hold on;
%         end
%     end
%     
%     %%
% %     fig = [h_gl h_gr h_ur h_r h_el h_l hr_NLS hl_NLS hur_NLS hul_NLS];
% %     fig = [hr_NLS hl_NLS hur_NLS hul_NLS];
% %     legend(fig,gl,gr,ur,r,el,l,r_NLS,l_NLS,ur_NLS,ul_NLS);
% %     [leg_NLS, obj2, p2, str2] = legend(fig,r_NLS,l_NLS,ur_NLS,ul_NLS);
% end