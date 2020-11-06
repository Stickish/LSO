mex gsp.c

A = [];
for i = 1 : 10
   A = [A; i i-1]; 
   
end

u = [1; -1; 3; -2];

if u(:) < 0
        u(:) = -u(:);
end
u
length(A(1,:))

%disp(A)