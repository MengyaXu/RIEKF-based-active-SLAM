clear;
clc;

%%
round = 0;
for mode = [1 2 3 5]
    round = round + 1;
%     if round == 3
%         op = 1;
%     else
%         op = 4;
%     end
    op = 4;

    if mode == 1
        resultId = 1;
        str = 'EKF';
%         s1 = 'EKF';
        s1 = 'greedy+EKF';
        setting_er = 'k-';
        setting_ef = 'k*';
        setting_tp = 'k-';
        setting_tu = 'k-';
    elseif mode == 3
        if op == 1
            resultId = 2;
        else
            resultId = 4;
        end
        str = 'IEKF';
%         s3 = 'RIEKF';
        s3 = 'greedy+RIEKF';
        setting_er = 'r-';
        setting_ef = 'r*';
        setting_tp = 'r-';
        setting_tu = 'r-';
    elseif mode == 2
        resultId = 2;
        mode = 2;
        op_2 = 1;
%         str = 'IEKF';
%         s2 = 'predetermined+RIEKF';
        str = 'NLSI';
        s2 = str;
        setting_er = 'b-';
        setting_ef = 'b*';
        setting_tp = 'b-';
        setting_tu = 'b-';
    elseif mode == 5
        resultId = 2;
%         mode = 2;
%         op_2 = 2;
        str = 'NLSlb';
        s5 = str;
        setting_er = 'c-';
        setting_ef = 'c*';
        setting_tp = 'c-';
        setting_tu = 'c-';
    elseif mode == 4
        resultId = 2;
        str = 'NLSall';
        s4 = 'NLS';
        setting_er = 'b-';
        setting_ef = 'b*';
        setting_tp = 'b-';
        setting_tu = 'b-';
    end
    if op == 1
        str_op = 'pre_';
    elseif op == 4
        str_op = '';
    end
    fer = fopen(['result/rand_' str_op str '_er_' num2str(resultId) '.txt'], 'r');
    fef = fopen(['result/rand_' str_op str '_ef_' num2str(resultId) '.txt'], 'r');
    ftp = fopen(['result/rand_' str_op str '_tp_' num2str(resultId) '.txt'], 'r');
    ftu = fopen(['result/rand_' str_op str '_tu_' num2str(resultId) '.txt'], 'r');

    % fp = fopen(['result/rand_' str_op str '_p_' num2str(resultId) '.txt'], 'r');
    % fxa = fopen(['result/rand_' str_op str '_xa_' num2str(resultId) '.txt'], 'r');
    % fxr = fopen(['result/rand_' str_op str '_xr_' num2str(resultId) '.txt'], 'r');
    % fxf = fopen(['result/rand_' str_op str '_xf_' num2str(resultId) '.txt'], 'r');
    % fxls = fopen(['result/rand_' str_op str '_xls_' num2str(resultId) '.txt'], 'r');
    %%
    er = [];
    tp = [];
    tu = [];
    ef = [];
    er(1) = 0;
    er_min = inf;
    er_max = 0;
    ef_min = inf;
    ef_max = 0;
    tp_min = inf;
    tp_max = 0;
    tu_min = inf;
    tu_max = 0;
    i = 0;
    line = 1.5;
    for j = 2: 500
        erline = fgetl(fer);
        if erline ~= -1
            i = i + 1;
            er(j) = str2double(erline);
            er(j) = real(sqrt(er(j)));
            if mode == 5 && er(j) > 1
                er(j) = er(j - 1);
            end
            if er(j) > er_max
                er_max = er(j);
            end
            if er(j) < er_min
                er_min = er(j);
            end
        end

        tpline = fgetl(ftp);
        if tpline ~= -1
            tp(j - 1) = str2double(tpline);
            if tp(j - 1) < tp_min
                tp_min = tp(j - 1);
            end
            if tp(j - 1) > tp_max
                tp_max = tp(j - 1);
            end
        end
        tuline = fgetl(ftu);
        if tuline ~= -1
            tu(j - 1) = str2double(tuline);
            if tu(j - 1) < tu_min
                tu_min = tu(j - 1);
            end
            if tu(j - 1) > tu_max
                tu_max = tu(j - 1);
            end
        end
    end
    if mode == 1
        figure(1)
        her1 = plot(1:i+1, er, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(m)');
        set(gca,'FontSize',16); 
%         axis([0 500 0 8]);
        hold on;
        figure(2)
        htp1 = plot(1:i, tp, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(s)');
        set(gca,'FontSize',16); 
        hold on;
        figure(3)
        htu1 = plot(1:i, tu, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(s)');
        set(gca,'FontSize',16); 
        hold on;
    elseif mode == 2
        figure(1)
        her2 = plot(1:i+1, er, setting_er, 'LineWidth',line);
        hold on;
        figure(2)
        htp2 = plot(1:i, tp, setting_er, 'LineWidth',line);
        hold on;
        figure(3)
        htu2 = plot(1:i, tu, setting_er, 'LineWidth',line);
        hold on;
    elseif mode == 3
        figure(1)
        her3 = plot(1:i+1, er, setting_er, 'LineWidth',line);
        hold on;
        figure(2)
        htp3 = plot(1:i, tp, setting_er, 'LineWidth',line);
        hold on;
        figure(3)
        htu3 = plot(1:i, tu, setting_er, 'LineWidth',line);
        hold on;
    elseif mode == 4
        figure(1)
        her4 = plot(1:i+1, er, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(m)');
        set(gca,'FontSize',16); 
        hold on;
        figure(2)
        htp4 = plot(1:i, tp, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(s)');
        set(gca,'FontSize',16); 
        hold on;
        figure(3)
        htu4 = plot(1:i, tu, setting_er, 'LineWidth',line);
        xlabel('x(step)');
        ylabel('y(s)');
        set(gca,'FontSize',16); 
        hold on;
    elseif mode == 5
        figure(1)
        her5 = plot(1:i+1, er, setting_er, 'LineWidth',line);
        axis([0 500 0 6]);
        hold on;
        figure(2)
        htp5 = plot(1:i, tp, setting_er, 'LineWidth',line);
        hold on;
        figure(3)
        htu5 = plot(1:i, tu, setting_er, 'LineWidth',line);
        hold on;
    end
step = i + 1;
    %%
    efline = fgetl(fef);
    j = 1;
    P = [];
    while efline ~= -1
        ef(:, j) = str2num(efline)';
        ef(2, j) = real(sqrt(ef(2, j)));
        if ef(2, j) > ef_max
            ef_max = ef(2, j);
        end
        if ef(2, j) < ef_min
            ef_min = ef(2, j);
        end
        figure(4)
        hef1 = plot(ef(1, j), ef(2, j), setting_ef);
        hold on;

        efline = fgetl(fef);
        j = j + 1;
    end

    str
    ser = sum(er) / step
    maer = er_max
    mier = er_min

    stp = sum(tp)
    matp = tp_max
    mitp = tp_min

    stu = sum(tu)
    matu = tu_max
    mitu = tu_min
    
%     if mode ~= 1 || op ~= 1
        sef = sum(ef(2, :))/j
        maef = ef_max
        mief = ef_min
%     end

    fclose(fer);
    fclose(fef);
    fclose(ftp);
    fclose(ftu);
end
%%
% preFigEr = [her1, her4, her3];
% preFigTp = [htp1, htp4, htp3];
% preFigTu = [htu1, htu4, htu3];
% legend(preFigEr, s1, s4, s3);
% legend(preFigTp, s1, s4, s3);
% legend(preFigTu, s1, s4, s3);

% preFigEr = [her4, her3];
% preFigTp = [htp4, htp3];
% preFigTu = [htu4, htu3];
% legend(preFigEr, s4, s3);
% legend(preFigTp, s4, s3);
% legend(preFigTu, s4, s3);

% preFigEr = [her1, her2, her3, her5];
% preFigTp = [htp1, htp2, htp3, htp5];
% preFigTu = [htu1, htu2, htu3, htu5];
% legend(preFigEr, s1, s2, s3, s5);
% legend(preFigTp, s1, s2, s3, s5);
% legend(preFigTu, s1, s2, s3, s5);

% preFigEr = [her1, her3, her2];
% preFigTp = [htp1, htp3, htp2];
% preFigTu = [htu1, htu3, htu2];
% legend(preFigEr, s1, s3, s2);
% legend(preFigTp, s1, s2, s3);
% legend(preFigTu, s1, s2, s3);

