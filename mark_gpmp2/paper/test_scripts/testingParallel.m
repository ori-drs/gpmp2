A = rand(1,1000);
B = rand(1,1000);
C = zeros(1,1000);
D = zeros(1,1000);

tic;
for i = 1:1000
    C(i) = A(i) + B(i);
end
toc;

tic;
parfor i = 1:1000
    D(i) = A(i) + B(i);
end
toc;