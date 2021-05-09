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
phi = state(7);
theta = state(8);
psi = state(9);
p = state(10);
q = state(11);
r = state(12);

R = EULERXYZ([phi;theta;psi]);
% Acceleration
acc = ([0; 0; -cf.mass * cf.g]+R*[0;0;u1])/cf.mass;

% Angular velocity
phi_dot = p;
theta_dot = q;
psi_dot = r;

%Angular acceleration
omega = [p;q;r];
omega_dot = cf.invI * (u2 - cross(omega, cf.I*omega));

state_dot = zeros(12,1);
state_dot(1)  = xdot;
state_dot(2)  = ydot;
state_dot(3)  = zdot;
state_dot(4)  = acc(1);
state_dot(5)  = acc(2);
state_dot(6)  = acc(3);
state_dot(7)  = phi_dot;
state_dot(8)  = theta_dot;
state_dot(9)  = psi_dot;
state_dot(10) = omega_dot(1);
state_dot(11) = omega_dot(2);
state_dot(12) = omega_dot(3);

end

