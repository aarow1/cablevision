addpath('matlab_tools');

H = eye(4);
H(1:3, 4) = [1, 1, 1]';
plot_H(H);