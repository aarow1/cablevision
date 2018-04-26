function [v] = vee(skew)
%VEE returns the vee map of a 3x3 matrix
%   turns a skew symetrix vector into a 3x1 vector 

if (size(skew) ~= [3 3])
    error('input is not the right size, must be 3x3')
end

if(skew' ~= -skew)
    error('input is not skew symmetric')
end

v = [skew(3,2) skew(1,3) skew(2,1)]';

end

