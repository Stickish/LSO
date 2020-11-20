function [d] = countRepeats(a,k)
%countRepeats Finds number of times number k is repeated in a vector a
d=0;
for i=1:size(a,1)
    if(a(i)== k)
        d=d+1;
    end
end


