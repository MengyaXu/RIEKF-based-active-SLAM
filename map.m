
mode = 3;
if mode ==1
    % % spars
    L = [3 6 3 12 7 8 0 10 15 2 10 15];     % landmarks [x1 y1 x2 y2...]
elseif mode == 2
    % circle
    phi = [0:pi/10:2*pi-pi/10];
    for j = 1:k0
        L0(2*j-1:2*j) = [10*cos(phi(j)) 10*sin(phi(j))+10];
    end
elseif mode == 3
    % % four corners
    L0 = [];
    Lx = [5:5:25, 75:5:95];
    Ly = [5:5:25, 75:5:95];
    % obstacle = [90 80 8 85 15 5]';
    obstacle = [];
    for i = Lx
        for j = Ly
            ob = 0;
            for n = 1:3:length(obstacle)
                delta = [i,j]' - obstacle(n:n+1);
                d = real(sqrt(delta' * delta));
    %             obstacle(n+2)
                if d <= obstacle(n+2)
                    ob = 1;
                end
            end
            if ob == 0
                L0 = [L0 i j];
            end
        end
    end
elseif mode == 4
    % random
    % obstacle = [90 80 8 85 15 5]';
    obstacle = [];
    k = 50;
    L0 = randi([1, 100], 1, 2*k);
end
%%
obstacle = [];
for j = 1:2:length(L0)
    obstacle = [obstacle' L0(j:j+1) 0]';
end

save map L0 obstacle