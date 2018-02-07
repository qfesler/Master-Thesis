clear all
close all
clc
csvfiles = dir('*.csv');
means = [];
realValues = [];

realValue = 0.6;
for file = csvfiles'
    realValues = [realValues,realValue];
    datas = csvread(file.name);
    datacorr = [];
    for i = datas
       if i ~= 0
          datacorr = [datacorr,i]; 
       end
    end
    means = [means,mean(datacorr)];
    realValue = realValue + 0.1;
end

plot(realValues)
hold on
plot(means,'r')