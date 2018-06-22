close all
filename = "10_27";
M = csvread(filename+".csv");
plot(M(:,1),M(:,2),'*')
k = waitforbuttonpress;
A = size(M);
i = 1;
while i <= A(1)
    row = M(i,:);
    if (abs(row(1)) > 4000) ||  (abs(row(1)) > 3000)
        M(i,:) = [];
        i = i-1;
    end
    A = size(M);
    i = i+1;
end

plot(M(:,1),M(:,2),'*')

%close all;[maxVal, row2del] = max(M(:,2)); M(row2del,:) = []; plot(M(:,1),M(:,2),'*'); save(filename,'M')

%close all;[maxVal, row2del] = min(M(:,1)); M(row2del,:) = []; plot(M(:,1),M(:,2),'*'); save(filename,'M')
k = waitforbuttonpress;

save(filename,'M')

close all