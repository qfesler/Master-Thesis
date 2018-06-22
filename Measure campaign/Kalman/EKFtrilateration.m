clear all;close all;

load distdata.mat;

%% TODO Load measureddist
%% TODO Offset
Offset = 1;

%% Coord beacons

X1 = 0;
Y1 = 0;
X2 = 0;
Y2 = 2000;
X3 = 2800;
Y3 = 1000;

%% INIT


%System Constants
% x = Ax + Bu (Bu unknown)
% z = Hx

%Time between 2 measures = 10ms
DT = 0.1;

v = 400; %mm/s
% states : x, y, xdot, ydot, theta
x0 = [1400; 1600; -v; 0];

%environemental errors (here, we put the impact of input we don't know)
P0=[10 0 0 0 ;...
   0 10 0 0 ;...
   0 0 10 0 ;...
   0 0 0 10 ];

%Q hard to choose
Q=[1 0 0 0 ;...
   0 1 0 0 ;...
   0 0 1 0 ;...
   0 0 0 1 ];

% R noise in the system (cov of measure)
R=[100 0 0;...
   0 100 0;...
   0 0 100];


x = x0;
P = P0;


%% KALMAN (for loop)

Kalmanpos = [];
computedpos = [];
lines = @(x) size(x,1); % lines(x) calculates number of lines in matrix x
for i = 1 : lines(measureddist(Offset:end,:))
    measure = measureddist(i+Offset-1,:);
    measure = measure';
    
    A = [ 1 0 DT 0;...
          0 1 0 DT;...
          0 0 1 0;...
          0 0 0 1];
          %0 0 -x(4)/(x(4)^2 + x(3)^2) x(3)/(x(3)^2 + x(4)^2) 0];
    dist1 = [x(1)-X1;x(2)-Y1];
    dist2 = [x(1)-X2;x(2)-Y2];
    dist3 = [x(1)-X3;x(2)-Y3];
          
    H = [dist1(1)/norm(dist1) dist1(2)/norm(dist1) 0 0 ;...
         dist2(1)/norm(dist2) dist2(2)/norm(dist2) 0 0 ;...
         dist3(1)/norm(dist3) dist3(2)/norm(dist3) 0 0 ];
          
          
    Xpredict = [x(1) + DT*x(3);...
                x(2) + DT*x(4);...
                x(3);...
                x(4)];
                %atan(x(4)/x(3))];
                
    h = [norm(dist1);...
         norm(dist2);...
         norm(dist3)];
            
    Ppredict = A*P*A' + Q;
    y = measure - h;
    S = H*Ppredict*H' + R;
    K = Ppredict* H' / S;
    x = Xpredict + K*y;
    P = (eye(4)- K*H)*Ppredict;
    computedpos = [computedpos trilatmatlab(measure)];
    Kalmanpos = [Kalmanpos ; x'];
end
measuredpos = computedpos';
followline