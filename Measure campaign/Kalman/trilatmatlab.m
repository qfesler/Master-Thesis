function position = trilatmatlab(measure)

r = measure;
p1 = [0;0];
p2 = [0;2000];
p3 = [2800;1000];
p = [p1 p2 p3];

a = 0;
ppTp = 0;
for i = [1,2,3]
    ppTp = ppTp +r(i)^2 * p(:,i);
    a = a + ( p(:,i)*p(:,i)'*p(:,i) - r(i)^2 * p(:,i) )/3;
end

B = 0;

for i = [1,2,3]
    B = B + (-2* p(:,i)*p(:,i)' - p(:,i)'*p(:,i)*eye(2) + r(i)^2 * eye(2))/3;
end

c = 0;

for i = [1,2,3]
    c = c+p(:,i);
end
c = c/3;

f = a + B*c + 2*(c*c')*c;



H = 0;

for i = [1,2,3]
    H=H+p(:,i)*p(:,i)';
end
H = -2/3*H + 2*(c*c');

q = -H\f;

position = [c(1)+q(1) ; c(2)+q(2)];

end