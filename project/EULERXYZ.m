function [RXYZ] = EULERXYZ(theta)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

Rx = ROTX(theta(1));
Ry = ROTY(theta(2));
Rz = ROTZ(theta(3));
RXYZ = Rx*Ry*Rz;
end

