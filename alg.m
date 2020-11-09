%%
clear, clc
% 
% dimX = 8;
% dimY = 6;
% k = 3;
% com = [1 48; 2 42; 3 43];
% initilization
theta = 2;
VLSI_8_6_7;
u = ones(dimX*dimY*2,1001)./k;
mex gsp.c;
h = zeros(1000,1);
alpha = zeros(1000,1);
hs = k;

% for loop
for t = 1 : 1000

    % solving sub prob with gsp and cost
    pi = u(:,t);
    nl = gsp(dimX,dimY,pi,k,com);

    % Calculate cost per route; remove route with
    % cost > 1 (required routes stored in nl and pairs in com)
    h(t) = sum(pi);
    okcom = [];
    newnl = [];
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
%     for i = 1 : dimX*dimY*2
%         gamma(i) = 1;   
%         last = 0;
%         for l = 1 : length(okcom)
%             first = last+1;
%             slask = find(newnl(last+1:length(newnl)) == okcom(l));
%             last = slask(1)+first-1;
%             gamma(i) = gamma(i) - (length(newnl(first:last)) - 1);
%         end
%     end 
    
    for i = 1 : dimX*dimY*2
        gamma(i) = 1;   
        last = 0;
        for l = 1 : length(okcom)
            first = last+1;
            slask = find(newnl(first:length(newnl)) == com(okcom(l),1));
            last = slask(1)+first-1;
            
            if ismember(i,newnl(first:last)) == true
                gamma(i) = gamma(i) - 1;
            end
        end
    end 

    % h* = k
    alpha(t) = theta*(hs-h(t))/norm(gamma)^2;
    
    % update u and theta

    u(:,t+1) = u(:,t) + gamma*alpha(t);
    % projection
    for i = 1 : length(u(:,t))
        if u(i,t+1) < 0
            u(i,t+1) = 0;
        end
    end
    
    if mod(t,10) == 0
       theta = theta*0.95; 
    end
    
    %
    x = zeros(dimX*dimY*2,dimX*dimY*2,k);
    last = 0;
    for m=1:length(okcom)
        first = last+1;
        slask = find(newnl(first:length(newnl)) == com(okcom(m),1));
        last = slask(1)+first-1;
        for i=first:last-1
            x(newnl(i),newnl(i+1),m) = 1;
            x(newnl(i+1),newnl(i),m) = 1;
        end
    end
    
    
    
    % end for loop and update t
end

upper_bound = min(h)

shift = 25;
visagrid(dimX,dimY,newnl,com,pi,shift);

%%
clear;
clc;

dimX = 8;
dimY = 6;
shift = 25;
pip = ones(dimX*dimY*2,1)*0.1;
newnl = [48 47 46 45 44 43 42 41 54 53 52 51 50 49 1 42 60 59 58 57 56 55 2 43 66 65 64 63 62 61 3];
com = [1 48; 2 42; 3 43];
visagrid(dimX,dimY,newnl,com,pip,shift);
