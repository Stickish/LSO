%%
clear, clc

% initilization
theta = 2;
VLSI_8_6_7;
u = zeros(dimX*dimY*2,1000);
mex gsp.c;
h = zeros(1000,1);

% for loop
for t = 1 : 1000

    % solving sub prob with gsp and cost
    pi = u(:,t);
    nl = gsp(dimX,dimY,pi,k,com);

    % Calculate cost per route; remove route with
    % cost > 1 (required routes stored in nl and pairs in com)
    h(t) = sum(pi);
    last = 0;
    for i = 1 : k
        first = last+1;
        slask = find(nl(last+1:length(nl)) == com(i,1));
        last = slask(1)+first-1;
        if (sum(pi(nl(first:last))) < 1)
            okcom = [okcom i]; 
            newnl = [newnl; nl(first:last)];
            h(t) = h(t) + 1 - sum(pi(nl(first:last)));
        end
    end

    % calculate dir and lenght
    
    gamma = zeros(dimX*dimY*2,1);
    for i = 1 : dimX*dimY*2
        gamma(i) = 1;   
        for l = okcom(1) : okcom(lenght(okcom))
            gamma(i) = gamma(i) - (lenght(newnl(l,:)) - 1);
        end
    end 
    
    % h* = k
    alpha = theta*(k-h(t))/norm(gamma)^2;
    
    % update u and theta

    u(:,t+1) = u(:,t) + gamma*alpha;
    % projection
    for i = 1 : lenght(u(:,t))
        if u(i,t+1) < 0
            u(i,t+1) = 0;
        end
    end
    
    if mod(t,10) == 0
       theta = theta*0.95; 
    end
    
    % end for loop and update t
end