%% Subgrad + Heuristic
clear, clc

% time variable tic, outer for loop for time avrage calculations.
tic;
for g = 1:5

% initilization
theta = 1.95;
VLSI_8_6_7;
u = ones(dimX*dimY*2,1)./k;
mex gsp.c;
ti = 1000;
h = zeros(ti,1);
z = zeros(ti,1);
alpha = zeros(ti,1);
hs = 0;

% for loop
for t = 1 : ti
    
    % Subgradient calculations

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

    % calculate subgradient (gamma) with countRepeats
    gamma = zeros(dimX*dimY*2,1);
    for i = 1:dimX*dimY*2
        gamma(i) = (1 - countRepeats(newnl,i));
    end

    % calculate step length (alpha)
    alpha(t) = (theta * (h(t) - hs))/sum((gamma).^2); 
    
    % update u and project it onto the feasible set
    for i = 1:size(u,1)
        u(i) = max(0,u(i) - alpha(t)*gamma(i));   
    end  
    
    % update theta
    if mod(t,10) == 0
       theta = theta*0.95; 
    end
    
    % Heuristic calculations
    
    % converte newnl to x_{ijl} matrix 
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
    
    % calculate x_bar from alpha and x_{ijl}
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
    
    % create x_heru from x_bar where x_heru is binary
    % (in each col, set only the largest pos value to 1, others to 0)
    x_heru = zeros(dimX*dimY*2,dimX*dimY*2,k);
    for l = 1:length(okcom)
        for i = 1:dimX*dimY*2
            [m,j] = max(x_bar(i,:,l));
            if m > 0.01
                x_heru(i,j,l) = 1;
            end
        end
    end

    % calculate z and convert x_heru to a vector with the conections
    % (nl_heru), using a greedy alg., try to find a conection between node
    % start_l and end_l such that no node is used twice. If such a
    % conection is found, add the conection and increase z.
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

    % calculate lower bound and save best nl_heru
    lower_bound = max(z);
    if z(t) == lower_bound
        nl_h = nl_heru;
    end
    
    % end for loop and update t
end
end

% time average
average_time = toc/5

% calculate a upper bound and display upper and lower bound
upper_bound = min(h)
disp(lower_bound);

%% plot newnl
% plot the solution from the subgradient algorithm

shift = 25;
visagrid(dimX,dimY,newnl,com,u,shift);

%% plot heru
% plot the solution from the heuristic

shift = 25;
com_h = [];
start = 1;
% find the conections which are satisfied in the heuristic
for l = 1:length(okcom)
    if ismember(com(l,1),nl_h(start:length(nl_h))) == true && ismember(com(l,2),nl_h(start:length(nl_h))) == true
        com_h = [com_h; com(l,:)];
        slask = find(nl_h(start:length(nl_h)) == com(l,1));
        start = start + slask(1);
    end
end

visagrid(dimX,dimY,nl_h,com_h,u,shift);

%% Plot dual obj. val. h(t)
% plot the dual ojective value (h) and the heuristic objective value as a
% function of the number of iterations.
clf

t = 1:ti;
plot(t,h);
hold on;
plot(t,z);
