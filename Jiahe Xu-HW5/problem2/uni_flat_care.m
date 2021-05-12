function f = uni_flat_care()
% boundary conditions in state space
x0 = [0; 5; 0];
xm = [5; 2.5; 0];
xf = [0; 0; 0];
T = 10;

%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%
S.u1 = 1;
y0 = uni_h(x0);
ym = uni_h(xm);
yf = uni_h(xf);

% desired path1
dy0 = S.u1*[cos(x0(3)); sin(x0(3))]; % desired starting velocity
dym = S.u1*[cos(xm(3)); sin(xm(3))]; % desired middle velocity
A = poly3_coeff(y0, dy0, ym, dym, T);
X = A*poly3([0:.01:T]);
plot(X(1,:), X(2,:), '-r')
hold on
%%%%%%%%% TRAJECTORY TRACKING 
S.A = A;
x = x0+[0.25 ; .25;.1];
% simulate system
[ts, xs] = ode45(@uni_ode, [0 T], x, [], S);
plot(xs(:,1), xs(:,2), '-b');
hold on


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% desired path2
S.u1 = -1;
dym = S.u1*[cos(xm(3)); sin(xm(3))]; % desired starting velocity
dyf = S.u1*[cos(xf(3)); sin(xf(3))]; % desired middle velocity
A = poly3_coeff(ym, dym, yf, dyf, T)
X = A*poly3([0:.01:T]);
plot(X(1,:), X(2,:), '-r')
hold on
%%%%%%%%% TRAJECTORY TRACKING 
S.A = A;
x = xs(end,:)'
% simulate system
[ts, xs] = ode45(@uni_ode, [0 T], x, [], S);
plot(xs(:,1), xs(:,2), '-b');
size(xs)
legend('desired', 'executed')


function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = uni_h(x)
% output function
y = x(1:2);

function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];

function f = dpoly3(t)
f = [3*t.^2; 2*t; ones(size(t)); zeros(size(t))];

function f = d2poly3(t)
f = [6*t; 2; zeros(size(t)); zeros(size(t))];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function u = uni_ctrl(t, x, S)
% tracking control law

% get desired outputs (x-y positions, velocities, accelerations)
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

u1d = sign(S.u1)*norm(dyd);
u2d = (d2yd(2)*dyd(1) - d2yd(1)*dyd(2))/(u1d^3);
x3d = atan( dyd(2)/dyd(1) ) ;

ud = [u1d; 
      u2d;];
  
xd = [yd; x3d];

A=[
    0, 0, -u1d*sin(x3d);
    0, 0,  u1d*cos(x3d);
    0, 0,           0;];
l = 1;
B= [  
    cos(x3d),                      0;
    sin(x3d),                      0;
    tan(u2d)/l, (u1d*(tan(u2d)^2 + 1))/l;];

[X,L,K] = care(A, B, eye(3));

% set control law
u = ud - K*(x - xd);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dq = uni_ode(t, q, S)
% unicycle ODE
u = uni_ctrl(t, q, S);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% add noise
% ii_1 u+ 0.001*randn(2,1);
% ii_2 u+ 0.01*randn(2,1);
% ii_3 u+ 0.05*randn(2,1);
% ii_4 u+ 0.1*randn(2,1);
% higher variance will make the trajectory unstable
% and longer time to simulate the trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u = u+ 0.1*randn(2,1);
dq = [cos(q(3))*u(1);
       sin(q(3))*u(1);
       u(1)*tan( u(2) )];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
