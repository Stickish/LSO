%%
clear, clc

% initilization
theta = 2;
VLSI_8_6_7;
u = zeros(dimX*dimY*2);
mex gsp.c;

% for loop
for t = 1 : 1000

    % solving sub prob with gsp and cost

    nl = gsp(dimX,dimY,u,k,com);

    % Calculate cost per route; remove route with
    % cost > 1 (required routes stored in nl and pairs in com)
    h = sum(u);
    last = 0;
    for i = 1 : k
        first = last+1;
        slask = find(nl(last+1:length(nl)) == com(i,1));
        last = slask(1)+first-1;
        if (sum(pi(nl(first:last))) < 1)
            okcom = [okcom i]; 
            newnl = [newnl; nl(first:last)];
            h = h + 1 - sum(pi(nl(first:last)));
        end
    end

    % calculate dir and lenght

    % update u and theta

    if mod(t,10) == 0
       theta = theta*0.95; 
    end
    
    % end for loop and update t
end