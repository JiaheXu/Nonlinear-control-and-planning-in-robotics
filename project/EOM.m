function [state_dot] = EOM(state,u1,u2)
% Author: Ray Zhang
% Calculate state_dot and state_ddot based on u1 and u2 calculated by
% controller 
cf = crazyflie();
 
x = state.pos(1);
y = state.pos(2);
z = state.pos(3);
xdot = state.vel(1);
ydot = state.vel(2);
zdot = state.vel(3);
phi = state.angle(7);
theta = state.angle(8);
psi = state.angle(9);
p = state.omega(10);
q = state.omega(11);
r = state.omega(12);

% Acceleration
acc = ([0; 0; -cf.mass * cf.grav]+R*[0;0;u1])/cf.mass;

% Angular velocity
phi_dot = p;
theta_dot = q;
psi_dot = r;

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
state_dot(7)  = phi_dot(1);
state_dot(8)  = theta_dot(2);
state_dot(9)  = psi_dot(3);
state_dot(10) = omega_dot(1);
state_dot(11) = omega_dot(2);
state_dot(12) = omega_dot(3);

end

