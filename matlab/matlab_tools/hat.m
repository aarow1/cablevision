function [skew] = hat(v)
%HAT returns the hat map of a 3x1 vector
%   turns a vector into a skew symmetric matrix

if (size(v) ~= [3 1])
    error('vector is not the right size, must be 3x1')
end

skew = [0   -v(3)  v(2);...
        v(3)   0  -v(1);...
       -v(2) v(1)    0];
end

