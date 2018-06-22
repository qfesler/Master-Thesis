%clear all;close all;

%load positionsdata.mat;

%measuredpos = trajectory2;

%% INIT

%System Constants
% x = Ax + Bu (Bu unknown)
% z = Hx

%Time between 2 measures = 10ms
DT = 0.1;

% states : x, y, xdot, ydot
x0 = [1400; 1600; 0; 0];

%environemental errors (here, we put the impact of input we don't know)
P0=[10 0 0 0;...
   0 10 0 0;...
   0 0 10 0;...
   0 0 0 10];

%Q hard to choose
Q=[10 0 0 0;...
   0 10 0 0;...
   0 0 1 0;...
   0 0 0 1];

% R noise in the system (cov of measure)
R=[100 0;...
   0 100];

A = [ 1 0 DT 0;...
      0 1 0 DT;...
      0 0 1 0;...
      0 0 0 1];
  
H = [1 0 0 0;...
     0 1 0 0];
 
x = x0;
P = P0;

%% KALMAN (for loop)
%measuredpos = computedpos';
Kalmanpos = [];
lines = @(x) size(x,1); % lines(x) calculates number of lines in matrix x
for i = 1 : lines(measuredpos)
    measure = measuredpos(i,:);
    measure = measure';
    
    Xpredict = A*x;
    Ppredict = A*P*A' + Q;
    y = measure - H*Xpredict;
    S = H*Ppredict*H' + R;
    K = Ppredict* H' / S;
    x = Xpredict + K*y;
    P = (eye(4)- K*H)*Ppredict;
    
    Kalmanpos = [Kalmanpos ; x'];
end

followline