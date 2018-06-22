close all
clear all

row = 2;
column = 1;

tablex = 0:10:300;
tabley = 0:-10:-200;

files = dir('*.mat');

varianceplot = zeros(21,31);
for datas = files'
    a = datas.name;
    [filepath,name,ext] = fileparts(a);
    load(a);
    varmat=var(M);
    varx = varmat(1);
    vary=varmat(2);
    vartot = vary+varx;
    coordinates = strsplit(name,'_');
    ycoord = str2double(coordinates(1));
    xcoord = str2double(coordinates(2));
    
    varianceplot(ycoord+1,xcoord+1) = vartot;
    varianceplot(end - (ycoord),xcoord+1) = vartot;
    
end

figure 
s1 = surf(tablex,tabley,varianceplot);
s1.EdgeColor ='none';

tablexsmooth = 0:1:300;
tableysmooth = 0:-1:-200;


[x, y] = meshgrid(tablex,tabley);

[xsmooth, ysmooth] = meshgrid(tablexsmooth,tableysmooth);

figure
ytest = 0:10:200;
ytestsmooth = 0:1:200;
variancesmooth = interp2(x, y, varianceplot,xsmooth,ysmooth);
%variancesmooth = interp2(varianceplot,zeros(201,301));
s2 = surf(tablexsmooth,tableysmooth,variancesmooth);
s2.EdgeColor ='none';

figure
[C,h] =  contourf(tablexsmooth,tableysmooth,variancesmooth);
clabel(C,h,'FontSize',14)
hold on
colorbar
caxis([0 6000])
ax = gca;
ax.FontSize = 14;
%plot3(x,y,varianceplot,'r*')