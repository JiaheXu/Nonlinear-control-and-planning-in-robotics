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
m=1;
J=0.1;
r = 0.2;
M = [m 0 0; 
     0 m 0; 
     0 0 J;];
D = [0.01 0 0; 
        0 0.1 0; 
        0 0 0.02;];
    
S.m = m;
S.r = r;
S.J = J;
S.M = M;
S.D = D;

% obstacle parameters
S.po = []; % position
%S.po = [1;1]; % position
S.ro = .25;   % obstacle radius
S.do = 1;    % sensing distance

% initial state
x0 = [3, 2, -pi/4,0 ,0, 0];

% simulate dynamics for 100 seconds
[ts, xs] = ode45(@point_ode, [0 100], x0,[],S);

plot(xs(:,1), xs(:,2),'.');


function mat = B(q,r)
rot = [cos(q) -sin(q) 0;
        sin(q) cos(q) 0;
        0 0 1];
mat =  rot * [1 1 0 ; 0 0 1; -r r 0];

function dx = point_ode(t, x, S)
q = x(1:3);
dq = x(4:6);
theta = x(3);
r = S.r;
J = S.J;
M = S.M;
D = S.D;
mat = B(theta,r);
u = [cos(theta)/2 sin(theta)/2  -1/(2*r); 
     cos(theta)/2 sin(theta)/2  1/(2*r); 
     -sin(theta)  cos(theta)         0];
u = inv(M)*mat*u*q;
kv = inv(M)*D;
ddq = -u- kv*dq;
dx = [x(4),x(5),x(6),ddq(1),ddq(2),ddq(3)]';
