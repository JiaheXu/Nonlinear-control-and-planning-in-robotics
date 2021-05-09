function [R] = ROTX(theta)
%   function to convert the rotation about x axis into a rotational matrix
%   the input is angle of rotation about x axis and the return is a the
%   rotational matrix 

R = [1,     0,              0;
     0,     cos(theta),     -sin(theta);
     0,     sin(theta),     cos(theta)];

end

