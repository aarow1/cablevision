function [handle] = plot_H(H,color)
%plot_H plots a homogeneous transformation

if (nargin == 1)
    color = [1 0 0];
end

origin = H(1:3,4);
origin_3x = [origin, origin, origin,];

handle = quiver3(origin_3x(1,:), ...
        origin_3x(2,:), ...
        origin_3x(3,:), ...
        H(1, 1:3)     , ...
        H(2, 1:3)     , ...
        H(3, 1:3)     , ...
        'maxheadsize', .5 ,...
        'linewidth',    1 ,...
        'color', color, ...
        'marker',  '.', ...
        'markersize', 20 ...
        );


end

