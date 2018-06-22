close all
%% arc function
circr = @(radius,rad_ang)  [radius*cos(rad_ang);  radius*sin(rad_ang)];
N = 25;

fig = figure;

%% first line
x0 = [1400;1600];
x1 = [400;1600];
traj = [x0 x1];
p1 = plot(traj(1,:),traj(2,:),'k','linewidth',3);
hold on

%% arc 1

r_angl = linspace(pi/2, pi, N);                             % Angle Defining Arc Segment (radians)
radius = 200;                                                   % Arc Radius
xy_r = circr(radius,r_angl);
plot(xy_r(1,:)+400, xy_r(2,:)+1400,'k','linewidth',3)

%% second line
x0 = [200;1400];
x1 = [200;700];
traj = [x0 x1];
plot(traj(1,:),traj(2,:),'k','linewidth',3)

%% arc 2
r_angl = linspace(pi, 3*pi/2, N);                             % Angle Defining Arc Segment (radians)
radius = 300;                                                   % Arc Radius
xy_r = circr(radius,r_angl);
plot(xy_r(1,:)+500, xy_r(2,:)+700,'k','linewidth',3)

%% line 3
x0 = [500;400];
x1 = [700;400];
traj = [x0 x1];
plot(traj(1,:),traj(2,:),'k','linewidth',3)

%% arc 3
r_angl = linspace(-pi/2, 0, N);                             % Angle Defining Arc Segment (radians)
radius = 200;                                                   % Arc Radius
xy_r = circr(radius,r_angl);
plot(xy_r(1,:)+700, xy_r(2,:)+600,'k','linewidth',3)

%% line 4
x0 = [900;600];
x1 = [900;900];
traj = [x0 x1];
plot(traj(1,:),traj(2,:),'k','linewidth',3)

%% arc 4
r_angl = linspace(pi/2, pi, N);                             % Angle Defining Arc Segment (radians)
radius = 400;                                                   % Arc Radius
xy_r = circr(radius,r_angl);
plot(xy_r(1,:)+1300, xy_r(2,:)+900,'k','linewidth',3)

%% line 5
x0 = [1300;1300];
x1 = [1400;1300];
traj = [x0 x1];
plot(traj(1,:),traj(2,:),'k','linewidth',3)

axis([0  2000    0  2000])

axis equal

sizematrix = size(measuredpos);
loops = sizematrix(1);
F(loops) = struct('cdata',[],'colormap',[]);

F(1) = getframe(fig);

for k = 1:sizematrix(1)
    p2 = plot(measuredpos(k,1),measuredpos(k,2),'r*','linewidth',2, 'MarkerSize',4);
    ax = gca;
    ax.FontSize = 14;
    p3 = plot(Kalmanpos(k,1),Kalmanpos(k,2),'b-','linewidth',3);
    %legend([p1 p2 p3],{'Real trajectory','Measures','Filtered results'})
    F(k) = getframe(fig);
end
%% To load positions from csv
% measuredpos = [];
% 
% for elem = positions
%     measuredpos = [measuredpos; eval(elem)];
% end

% p2 = plot(measuredpos(:,1),measuredpos(:,2),'r*','linewidth',2, 'MarkerSize',4);
% ax = gca;
% ax.FontSize = 14;
% %p2 = plot(computedpos(1,:),computedpos(2,:),'r*')
% p3 = plot(Kalmanpos(:,1),Kalmanpos(:,2),'b-','linewidth',3);
% legend([p1 p2 p3],{'Real trajectory','Measures','Filtered results'})



imwrite(im,map,'test.gif','DelayTime',0,'LoopCount',inf)