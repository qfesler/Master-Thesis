clear all
close all
%clc
csvfiles = dir('*.csv');
means = [];
realValues = [];
x = 0.2:0.1:5;
%x = 200:100:5000;
realValue = 0.2;
variance = [];
error = [];
for file = csvfiles'
    realValues = [realValues,realValue];
    datas = csvread(file.name);
    datacorr = [];
    for i = datas
       %if i ~= 0
          datacorr = [datacorr,i]; 
       %end
    end
    error = [error, mean(datacorr) - realValue];
    variance = [variance, var(datacorr)];
    means = [means,mean(datacorr)];
    realValue = realValue + 0.1;
end

plot(realValues,x,'linewidth',2)
hold on
plot(x,means,'r','linewidth',2)
xlabel('Real Distance')
ylabel('Measured Distance')
legend('Real distances','Measures', 'location','NorthWest')
ax = gca;
ax.FontSize = 12;