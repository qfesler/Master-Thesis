error = [];
for i = 1:length(realValues)
    error = [error, realValues(i)-means(i)];
end

plot(error)
