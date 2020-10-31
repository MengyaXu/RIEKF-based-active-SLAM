function [cv]=IEKF_GetCov(P,x,y)
s=1; k=40;   % original: k=20, changed by zhan

P=(P+P')/2; % add by Shoudong

if trace(P)<1e-5, r=zeros(3,3);
else
    %P
    r=real(sqrtm(P)); 
end

for j=1:k+1
  q=2*pi*(j-1)/k; 
  xi=s*3*r*[0;cos(q); sin(q)]; %I dont know why, but we can have a try!
  cv(:,j)=[xi(1)*y+xi(2);xi(1)*x+xi(3)]+[x;y];
end
plot(cv(1,:), cv(2,:), 'r');
% P is 3x3 covariance matrix, s is scale factor, x,y are estimated robot or landmark location

% A typical call is 

% CV=GetCov(sys.P(1:2,1:2,n),sys.X(1,n),sys.X(2,n));

% sys.P(1:2,1:2,n) is robot covariance, sys.X(1,n) sys.X(2,n) is robot location

% To plot it

% set(fig.car.CV,'xdata',CV(1,:),'ydata',CV(2,:));

% fig.car.CV is a graph handle
