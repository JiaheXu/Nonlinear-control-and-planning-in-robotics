function f = hw2_example()
clc;
clear;
% EN.530.678: Homework 2: working example
% simulation of point-mass system stabilization and obstacle avoidance
% The point-mass scenario is described by:
% state: x = (q,v) with configuration q=(px,py) and velocity v=(vx,vy)
% paramters: mass m
% obstacle: position po, radius ro, sensing distance do
% control gains: proportional kq, derivative kv, obstacle avoidance ko

% ----------------- main parameters --------------

% model parameters
S.m = 1;
 
% propotional control gains
S.kq = [1; 1];

% derivative control gains
S.kv = [2; 2];

% avoidance avoidance control gain
S.ko = .2;     

% obstacle parameters
S.po = [1;1]; % position
%S.po = []; % position
S.ro = .25;   % obstacle radius
S.do = 1;    % sensing distance

% initial state
x0 = [3, 2, -1, 0];

% simulate dynamics for 100 seconds
[ts, xs] = ode45(@point_ode, [0 100], x0, [], S);

% plot obstacle
ellipsoid(S.po(1),S.po(2), 0, S.ro, S.ro, 0);
hold on

% plot position path
plot(xs(:,1), xs(:,2), '.');



function u = point_ctrl(t, x, S)
% control law: u = k(t,x), where x=(q,v) is the state
% S is a structure containing all model parameters 
% see main file for their listing

p = x(1:2);  % position
v = x(3:4);  % velocity

% required spatial force
u = -S.kq.*p - S.kv.*v;

% add obstacle avoidance steering if an obstacle is given
if ~isempty(S.po)
  g = p - S.po;     
  d = norm(g) - S.ro;  % distance to obstacle
  g = d*g/norm(g);     % direction vector from obstacle to position
  
  a = -v'*g/(norm(v)*d); % cos of angle between velocity and
                         % obstacle direction
  if (d < S.do && a > 0) % if within sensing radius and heading towards obstacle
    u = u + S.ko/d*[0 -1; 1 0]*v;   % add steering control law
  end
end


function dx = point_ode(t, x, S)
% defines the ODE dx = f(t, x) for the fully-actuated hover ODE
% x = (q,v) denotes the full state, i.e. q=(px,py,theta), v=(vx,vy,w)

% position and orientation
p = x(1:2);

% control inputs from control law
u = point_ctrl(t, x, S);

% set derivative
dx = [x(3:4);
      1/S.m*u];
