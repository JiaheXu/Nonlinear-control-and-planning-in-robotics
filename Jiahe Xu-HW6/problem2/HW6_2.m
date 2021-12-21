function f = uni_flat_care()
clc;
clear;
% boundary conditions in state space
x0 = [-4 ; 0 ];
xf = [0 ; 0 ];
T = 10;


y0 = uni_h(x0);
yf = uni_h(xf);
dy0 = 8;
dyf = 0; 

% compute path coefficients
A = poly3_coeff(y0, dy0, yf, dyf, T);

% plot desired path
% the trajetory it too strange so this is NOT used
X1 = A*poly3([0:.01:T]);
X2 = zeros( size(X1) );
[m,n] = size(X1);
for i = 1 : n
    t = (i-1)*T/(n-1);
    yd = A*poly3(t);
    dyd = A*dpoly3(t);
    X2(1,i) = get_x2([yd,dyd]);
end
%plot(X1(1,:), X2(1,:), '-r');



%%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.A = A;
% 
% % perturb initial condition
x = x0;
% 
% % simulate system
[ts, xs] = ode45(@uni_ode, [0 T], x, [], S);


% 
% visualize
plot(xs(:,1), xs(:,2), '-b');
legend('executed trajectory')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x2 = get_x2(x)
% x(1) is x1
% x(2) is dx1
x2 = - 3/2*x(1)^2 - 1/2*x(1)^3 - x(2);

function dx1 = get_dx1(x)
dx1 = -x(2) - 3/2*x(1)^2 - 1/2*x(1)^3;

function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = uni_h(x)
% output function
y = x(1);


function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];

function f = dpoly3(t)
f = [3*t.^2; 2*t; ones(size(t)); zeros(size(t))];

function f = d2poly3(t)
f = [6*t; 2; zeros(size(t)); zeros(size(t))];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function u = uni_ctrl(t, x, S)
% tracking control law

% yd = S.A*poly3(t);
% dyd = S.A*dpoly3(t);
% d2yd = S.A*d2poly3(t);
%%%%%%%
% instead of tracking trajectory
% directly approaching the goal(origin in this case)
%%%
yd = 0;
dyd = 0;
d2yd = 0;


z1 = x(1) - yd;
z2 = get_dx1(x) - dyd;
%kp = 1 kd = 4
v = d2yd - 1*z1 - 4*z2;
u = - v - ( 3*x(1) + 3/2*x(1).^2 )*get_dx1(x);
%%%%%%%%
%output control
%%%%%%%%
% plot( t,u, '.r');
% hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dq = uni_ode(t, q, S)
% unicycle ODE
u = uni_ctrl(t, q, S);
dx1 = get_dx1(q);
dq = [dx1;
    u];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
