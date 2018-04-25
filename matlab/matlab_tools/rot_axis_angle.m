function [R] = rot_axis_angle(axis, angle)
%ROTATION_MATRIX returns the rotation around an axis by a given angle in
%radians
%   axis is a 3-d vector. Will be normalized in the function
%   angle is an angle in radians, rotation follows right hand rule

axis = axis / norm(axis);
ux = axis(1);
uy = axis(2);
uz = axis(3);
ca = cos(angle);
sa = sin(angle);

R = [...
      ca+ux*ux*(1-ca),    ux*uy*(1-ca)-uz*sa,   ux*uz*(1-ca)+uy*sa  ;...
      uy*ux*(1-ca)+uz*sa, ca+uy*uy*(1-ca),      uy*uz*(1-ca)-ux*sa  ;...
      uz*ux*(1-ca)-uy*sa, uz*uy*(1-ca)+ux*sa    ca+uz*uz*(1-ca)     ;...
    ];

end

