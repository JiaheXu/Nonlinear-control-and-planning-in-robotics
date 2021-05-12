function [RXYZ] = EULERXYZ(theta)

Rx = ROTX(theta(1));
Ry = ROTY(theta(2));
Rz = ROTZ(theta(3));
RXYZ = Rx*Ry*Rz;
end

