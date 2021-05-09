function [R] = ROTZ(theta)
%   function to convert the rotation about z axis into a rotational matrix
%   the input is angle of rotation about z axis and the return is a the
%   rotational matrix 

R = [cos(theta),    -sin(theta),    0;
     sin(theta),    cos(theta),     0;
     0,             0,              1];

end
