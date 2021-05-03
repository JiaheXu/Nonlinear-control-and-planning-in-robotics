function f = hw2_example()
clc;
clear;

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
% propotional control gains
S.kq = [1; 1];

% derivative control gains
S.kv = [2; 2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S.ko = 1;
% ko determines the ratio of changing direction
% when the robot sees an obstacle
% if it is too small, the robot might hit the obstacle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% obstacle parameters
%S.po = []; % position
S.po = [1;1]; % position
S.ro = .25;   % obstacle radius
S.do = 1;    % sensing distance

% initial state
x0 = [3, 2, -pi/4,0 ,0, 0];

% simulate dynamics for 100 seconds
[ts, xs] = ode45(@point_ode, [0 1000], x0,[],S);

% plot obstacle
ellipsoid(S.po(1),S.po(2), 0, S.ro, S.ro, 0);
hold on

% plot position path
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
% control inputs from control law
mat = B(theta,r);
u = [cos(theta)/2 sin(theta)/2  -1/(2*r); 
     cos(theta)/2 sin(theta)/2  1/(2*r); 
     -sin(theta)  cos(theta)         0];

%kp is for checking the value of v_dot
kp = inv(M)*mat* u;

u = inv(M)*mat*u*q;

kv = D;
n=1;
m=1;
%n and m are just parameters for better design of u
kp = n*kp;
kv = m*kv;

ze = [0 0 0;
      0 0 0;
      0 0 0];
id = [1 0 0;
      0 1 0;
      0 0 1];



p = q(1:2);
v = dq(1:2);
if ~isempty(S.po)
  g = p - S.po;     
  d = norm(g) - S.ro;  % distance to obstacle
  g = d*g/norm(g);     % direction vector from obstacle to position
  
  a = -v'*g/(norm(v)*d); % cos of angle between velocity and
                         % obstacle direction
  if (d < S.do && a > 0) % if within sensing radius and heading towards obstacle
      tmp = S.ko/d*[0 -1 0 ; 1 0 0 ; 0 0 0];
      kv = kv + tmp;   % add steering control law
  end
end

kv = inv(M)*kv;
ddq = -u- kv*dq;
A = [ ze , id; -kp, -kv ;];
v_dot = x'*(A+A')*x;

dx = [x(4),x(5),x(6),ddq(1),ddq(2),ddq(3)]';
