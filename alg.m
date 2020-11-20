%% Subgrad + Heuristic
clear, clc

tic;
for g = 1:5

% initilization
theta = 1.95;
VLSI_30_10_15;
u = ones(dimX*dimY*2,1)./k;
mex gsp.c;
ti = 1000;
h = zeros(ti,1);
z = zeros(ti,1);
alpha = zeros(ti,1);
hs = 0;

% for loop
for t = 1 : ti

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
    gamma = zeros(dimX*dimY*2,1);
    for i = 1:dimX*dimY*2
        gamma(i) = (1 - countRepeats(newnl,i));
    end

    alpha(t) = (theta * (h(t) - hs))/sum((gamma).^2); 
    for i = 1:size(u,1)
        u(i) = max(0,u(i) - alpha(t)*gamma(i));   
    end  
    
    if mod(t,10) == 0
       theta = theta*0.95; 
    end
    
    % Heuristic
    x = zeros(dimX*dimY*2,dimX*dimY*2,k);
    last = 0;
    for m=1:length(okcom)
        first = last+1;
        slask = find(newnl(first:length(newnl)) == com(okcom(m),1));
        last = slask(1)+first-1;
        for i=first:last-1
            x(newnl(i),newnl(i+1),m) = 1;
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
    
    % calculate z
%     x_heru = round(x_bar + 0.45);
    x_heru = zeros(dimX*dimY*2,dimX*dimY*2,k);
    for l = 1:length(okcom)
        for i = 1:dimX*dimY*2
            [m,j] = max(x_bar(i,:,l));
            if m > 0.01
                x_heru(i,j,l) = 1;
            end
        end
    end

    nl_heru = [];
    for l = 1:length(okcom)
        temp = com(okcom(l),2);
        nl_temp = temp;
        while temp ~= com(okcom(l),1)
            slask = find(x_heru(temp,:,l) == 1);
            b = 1;
            for i = 1:length(slask)
                if ismember(slask(i),nl_temp) == false && ismember(slask(i),nl_heru) == false
                    b = 2;
                    temp = slask(i);
                    nl_temp = [nl_temp temp];
                    break;
                end
            end
            if b == 1
                break;
            end
        end
        if b == 2
          z(t) = z(t) + 1;
          nl_heru = [nl_heru nl_temp];
        end
    end

    lower_bound = max(z);
    if z(t) == lower_bound
        nl_h = nl_heru;
    end
    
    % end for loop and update t
end
end

average_time = toc/5

upper_bound = min(h)
disp(lower_bound);

%% plot newnl

shift = 25;
visagrid(dimX,dimY,newnl,com,u,shift);

%% plot heru

shift = 25;
com_h = [];
for l = 1:length(okcom)
    if ismember(com(l,1),nl_h) == true && ismember(com(l,2),nl_h) == true
        com_h = [com_h; com(l,:)];
    end
end

visagrid(dimX,dimY,nl_h,com_h,u,shift);

%% Plot dual obj. val. h(t)
clf

t = 1:ti;
plot(t,h);
hold on;
plot(t,z);
