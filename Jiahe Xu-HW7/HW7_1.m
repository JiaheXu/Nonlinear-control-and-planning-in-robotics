function f = hw7_1()
clc;
clear;
% boundary conditions in state space
x0 = [-4 ; 0 ];
xf = [0 ; 0 ];
T = 10;

% % perturb initial condition
x = x0;
% 
% % simulate system
[ts, xs] = ode45(@uni_ode_bs, [0 T], x, []);
plot(xs(:,1), xs(:,2), '-b');
hold on;
%legend('backstepping trajectory')
[ts, xs] = ode45(@uni_ode_fl, [0 T], x, []);
%legend('feedback linearization trajectory');
plot(xs(:,1), xs(:,2), '-r');
legend('backstepping trajectory' ,'feedback linearization trajectory')

function y = uni_h(x)
% output function
y = x(1);

function dx1 = get_dx1(x)
dx1 = -x(2) - 3/2*x(1)^2 - 1/2*x(1)^3;

function u = uni_ctrl_fl(t, x)
yd = 0;
dyd = 0;
d2yd = 0;
z1 = x(1) - yd;
z2 = get_dx1(x) - dyd;
%kp = 1 kd = 4
v = d2yd - 1*z1 - 4*z2;
u = - v - ( 3*x(1) + 3/2*x(1).^2 )*get_dx1(x);
% plot(t,u, '.r');
% hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = uni_ctrl_bs(t, x)
k1 = 9;
k2 = 2;
x1 = x(1);
x2 = x(2);
u = k1 * ( -3/2*x1^2 - 1/2*x1^3 - x2 ) + x1 - k2*(x2 - 9*x1);
% plot(t,u, '.b');
% hold on;

function dq = uni_ode_bs(t, q)
% unicycle ODE
u = uni_ctrl_bs(t, q);
dx1 = get_dx1(q);
dq = [dx1;u];

function dq = uni_ode_fl(t, q)
% unicycle ODE
u = uni_ctrl_fl(t, q);
dx1 = get_dx1(q);
dq = [dx1;u];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
