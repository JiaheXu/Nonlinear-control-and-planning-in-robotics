function [state_dot] = EOM(state,u1,u2)
% Author: Ray Zhang
% Calculate state_dot and state_ddot based on u1 and u2 calculated by
% controller 
cf = crazyflie();
 
x = state(1);
y = state(2);
z = state(3);
xdot = state(4);
ydot = state(5);
zdot = state(6);
qW = state(7);
qX = state(8);
qY = state(9);
qZ = state(10);
p = state(11);
q = state(12);
r = state(13);

quat = [qW; qX; qY; qZ];
R = QuatToRot(quat); % convert quaternion to rotation matrix

% Acceleration
acc = ([0; 0; -cf.mass * cf.grav]+R*[0;0;u1])/cf.mass;

%-----------------------------------------------------------------
% Angular velocity
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;
%-----------------------------------------------------------------
%Angular acceleration
omega = [p;q;r];
omega_dot = cf.invI * (u2 - cross(omega, cf.I*omega));

state_dot = zeros(13,1);
state_dot(1)  = xdot;
state_dot(2)  = ydot;
state_dot(3)  = zdot;
state_dot(4)  = acc(1);
state_dot(5)  = acc(2);
state_dot(6)  = acc(3);
state_dot(7)  = qdot(1);
state_dot(8)  = qdot(2);
state_dot(9)  = qdot(3);
state_dot(10) = qdot(4);
state_dot(11) = omega_dot(1);
state_dot(12) = omega_dot(2);
state_dot(13) = omega_dot(3);

end

