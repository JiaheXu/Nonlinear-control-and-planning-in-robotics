function f = car_traj_bs()
clc;
clear;
% boundary conditions in state space
x0 = [-5; -3; 0.5 ; 1];
xf = [0; 0; 1 ; 1];
T = 10;
S.u1 = 1;
% boundary conditions in flat output space 
y0 = uni_h(x0);
yf = uni_h(xf);
dy0 = S.u1*[cos(x0(3)); sin(x0(3))]; % desired starting velocity
dyf = S.u1*[cos(xf(3)); sin(xf(3))]; % desired end velocity

% compute path coefficients
A = poly3_coeff(y0, dy0, yf, dyf, T);

% plot desired path
X = A*poly3([0:.01:T]);
plot(X(1,:), X(2,:), '-r')
hold on


%%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.A = A;
S.L = 1;
% gains
S.k = [1;1];

% perturb initial condition
x = x0 + [.25;.25;.1;0]

% % simulate system
[ts, xs] = ode45(@uni_ode, [0 T], x, [], S);
% 
% % visualize
plot(xs(:,1), xs(:,2), '-b');
% 
legend('desired', 'executed','Location','South')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)]
A = Y*inv(L);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = uni_h(x)
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

% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% get current output
y = uni_h(x);

% velocity
x4 = x(4);
x3 = x(3);
% current velocity
dy = [cos(x3); sin(x3)]*x4;

% error state
e = y - yd;
de = dy - dyd;

k0 = 10;
k1 = 8;
z = k0*e + de;
A = [cos(x3) sin(x3);
    -sin(x3)/(x4^2) cos(x3)/(x4^2)];

ua = A*(-e - k0*de - d2yd - k1*z');
u = [ atan(ua(2) * S.L) ; ua(1)];

function dxa = uni_ode(t, x, S)

u = uni_ctrl(t, x, S);
%%noise
%noise = 0.9*randn(2,1);
%u = u + noise;

dxa = [cos(x(3))*x(4);
       sin(x(3))*x(4);
       tan(u(1))/S.L * x(4);
       u(2);];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
