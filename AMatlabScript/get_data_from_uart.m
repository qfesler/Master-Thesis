datas = zeros(1000,1);
s = serial('COM10');
fopen(s);
for i = 1:1000
    datas(i) = fgetl(s);
end
plot(datas);
%function to use with the data
histcounts

gfit