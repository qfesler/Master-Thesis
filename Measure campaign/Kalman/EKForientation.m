%clear all;close all;

%load positionsdata.mat;

%% INIT

%System Constants
% x = Ax + Bu (Bu unknown)
% z = Hx

%Time between 2 measures = 10ms
DT = 0.1;
Offset = 1;
v = 400; %mm/s
% states : x, y, xdot, ydot, theta
x0 = [1400; 1600; -v; 0; pi];

%environemental errors (here, we put the impact of input we don't know)
P0=[10 0 0 0 0;...
   0 10 0 0 0;...
   0 0 10 0 0;...
   0 0 0 10 0;...
   0 0 0 0 0.001];

%Q hard to choose
Q=[100 0 0 0 0;...
   0 100 0 0 0;...
   0 0 1 0 0;...
   0 0 0 1 0;...
   0 0 0 0 0.01];

% R noise in the system (cov of measure)
R=[250 0;...
   0 250];


  
H = [1 0 0 0 0;...
     0 1 0 0 0];
 
x = x0;
P = P0;


%% KALMAN (for loop)
%measuredpos = computedpos';
Kalmanpos = [];
lines = @(x) size(x,1); % lines(x) calculates number of lines in matrix x
for i = 1 : lines(measuredpos(Offset:end,:))
    measure = measuredpos(i+Offset-1,:);
    measure = measure';
    
    A = [ 1 0 DT 0 0;...
          0 1 0 DT 0;...
          0 0 0 0 -v*sin(x(5));...
          0 0 0 0 v*cos(x(5));...
          0 0 0 0 1];
          %0 0 -x(4)/(x(4)^2 + x(3)^2) x(3)/(x(3)^2 + x(4)^2) 0];
    
    Xpredict = [x(1) + DT*x(3);...
                x(2) + DT*x(4);...
                v*cos(x(5));...
                v*sin(x(5));...
                x(5)];
                %atan(x(4)/x(3))];
            
    Ppredict = A*P*A' + Q;
    y = measure - H*Xpredict;
    S = H*Ppredict*H' + R;
    K = Ppredict* H' / S;
    x = Xpredict + K*y;
    P = (eye(5)- K*H)*Ppredict;
    
    Kalmanpos = [Kalmanpos ; x'];
end

followline