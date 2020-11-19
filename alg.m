%%
clear, clc
% 
% dimX = 8;
% dimY = 6;
% k = 3;
% com = [1 48; 2 42; 3 43];
% initilization
theta = 1.95;
VLSI_8_6_7;
u = ones(dimX*dimY*2,1)./k;
mex gsp.c;
h = zeros(1000,1);
alpha = zeros(1000,1);
hs = 0;

% for loop
for t = 1 : 1000

    % solving sub prob with gsp and cost
    pi = u(:);
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
    
%     gamma = ones(dimX*dimY*2,1);
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
%     
%     for i = 1 : dimX*dimY*2 
%         last = 0;
%         for l = 1 : length(okcom)
%             first = last+1;
%             slask = find(newnl(first:length(newnl)) == com(okcom(l),1));
%             last = slask(1)+first-1;
%             
%             if ismember(i,newnl(first+1:last-1)) == true
%                 gamma(i) = gamma(i) - 1;
%             end
%         end
%     end 
%     
%     last = 0;
%     for l = 1 : length(okcom)
%         first = last+1;
%         slask = find(newnl(first:length(newnl)) == com(okcom(l),1));
%         last = slask(1)+first-1;
%             
%         for i = first+1:last
%             gamma(newnl(i)) = gamma(newnl(i)) - 1;
%         end
%     end

    gamma = zeros(dimX*dimY*2,1);
    for i = 1:dimX*dimY*2
        gamma(i) = (1 - countRepeats(newnl,i));
    end

    alpha(t) = (theta * (h(t) - hs))/sum((gamma).^2); 
    for i = 1:size(u,1)
        u(i) = max(0,u(i) - alpha(t)*gamma(i));   
    end  
    
%     % h* = k
%     alpha(t) = theta*(hs-h(t))/norm(gamma)^2;
%     
%     % update u and theta
% 
%     u(:,t+1) = u(:,t) + gamma*alpha(t);
%     % projection
%     for i = 1 : length(u(:,t))
%         if u(i,t+1) < 0
%             u(i,t+1) = 0;
%         end
%     end
    
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
            % x(newnl(i+1),newnl(i),m) = 1;
        end
    end
    
    if t == 1
        x_bar = x;
        x_temp = x_bar;
        alpha_bar = alpha(t);
    else
        alpha_temp = alpha_bar;
        alpha_bar = alpha_bar + alpha(t);
        x_bar = alpha_temp/alpha_bar * x_temp + alpha(t)/alpha_bar * x;
        x_temp = x_bar;
    end

    % end for loop and update t
end

% x_heru = round(x_bar + 0.45);
x_heru = zeros(dimX*dimY*2,dimX*dimY*2,k);
for l = 1:length(okcom)
    for i = 1:dimX*dimY*2
        [m,j] = max(x_bar(i,:,l));
        %if m > 0.1
            x_heru(i,j,l) = 1;
        %end
    end
end

nl_heru = [];
for l = 1:length(okcom)
    nl_heru = [nl_heru com(l,2)];
    temp = com(l,2);
    nl_temp = temp;
    while temp ~= com(l,1)
        slask = find(x_heru(temp,:,l) == 1);
        for i = 1:length(slask)
            if ismember(slask(i),nl_temp) == false
                temp = slask(i);
                nl_heru = [nl_heru temp];
                nl_temp = [nl_temp temp];
                break;
            end
        end
    end
end

% z_bar = 0;
% for l = 1:length(com(:,1))
%     if sum(x_bar(com(l,1),:,l)) ~= 0 && sum(x_bar(:,com(l,2),l)) ~= 0
%         z_bar = z_bar + 1; 
%     end
% end
    
upper_bound = min(h)

shift = 25;
visagrid(dimX,dimY,nl_heru,com,u,shift);

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
