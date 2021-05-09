function [R] = ROTY(theta)
%   function to convert the rotation about y axis into a rotational matrix
%   the input is angle of rotation about y axis and the return is a the
%   rotational matrix 

R = [cos(theta),    0,      sin(theta);
     0,             1,      0;
     -sin(theta),   0,      cos(theta)];

end
