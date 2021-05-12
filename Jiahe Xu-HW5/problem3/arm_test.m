function f = arm_test()
% model parameters
S.m1 = 1;
S.m2 = 1;
S.l1 = .5;
S.l2 = .5;
S.lc1 = .25;
S.lc2 = .25;
S.I1 = S.m1*S.l1/12;
S.I2 = S.m2*S.l2/12;
S.g = 9.8;

% initial state with high-velocity
x0 = [0; 0; 2; 5];
xf = [1; 1; 0; 0];
% desired state
S.xd = [1; 1; 0; 0];

y0 = uni_h(x0);
yf = uni_h(xf);
% desired accelration
S.ad = [0; 0];
T = 10; % simulation time
S.kp = 20; S.kd = 10;
% generate a trajectory
dy0 = [2; 5]; % desired starting velocity
dyf = [0; 0]; % desired middle velocity

A = poly3_coeff(y0, dy0, yf, dyf, T);
X = A*poly3([0:.01:T]);
[m,n] = size(X);
S.A = A;

%%%%%%%%%%%%%%%%%%%%%%%%
% output qd trajectory
%%%%%%%%%%%%%%%%%%%%%%%%
% plot(X(1,:), X(2,:), '-r')
% xlabel('q_1')
% ylabel('q_2')
% hold on


%%%%%%%%%%%%%%%%%%%%%%%%
% output ud
%%%%%%%%%%%%%%%%%%%%%%%%
% for t=0:.01:T
%     qd = S.A*poly3(t);
%     vd = S.A*dpoly3(t);
%     d2qd = S.A*d2poly3(t);
%     [Md, Cd, Nd] = arm_dyn(t, [qd,vd], S);
%     ud = Md*d2qd + Cd*vd + Nd;
%     plot(ud(1), ud(2), '*r')
%     hold on
% end
[ts, xs] = ode45(@arm_ode, [0 T], x0, [], S);

%%%%%%%%%%%%%%%%%%%%%%%%
% output simulated trajectory
%%%%%%%%%%%%%%%%%%%%%%%%
% plot(xs(:,1), xs(:,2), '-b')
% hold on
legend('desired', 'executed')



function u = arm_ctrl(t, x, S)
qd = S.A*poly3(t);
vd = S.A*dpoly3(t);
d2qd = S.A*d2poly3(t);

[Md, Cd, Nd] = arm_dyn(t, [qd,vd], S);
ud = Md*d2qd + Cd*vd + Nd;
plot(ud(1), ud(2), '*r')
hold on

q = x(1:2);
v = x(3:4);

[M, C, N] = arm_dyn(t, x, S);


virtual_input = d2qd - S.kd*(v-vd) - S.kp*(q-qd);
u = M*virtual_input + C*v + N;


function dx = arm_ode(t, x, S)
v = x(3:4);
[M, C, N] = arm_dyn(t, x, S);
u = arm_ctrl(t, x, S) ;
u = u + [0.2;0];

%%%%%%%%%%%%%%%%%%%%%%%%
% output control u with small external force
%%%%%%%%%%%%%%%%%%%%%%%%
plot(u(1), u(2), '*b')
hold on
    
dx = [v;
      inv(M)*(u - C*v - N)];

function [M, C, N] = arm_dyn(t, x, S)
q = x(1:2);
v = x(3:4);
c1 = cos(q(1));
c2 = cos(q(2));
s2 = sin(q(2));
c12 = cos(q(1) + q(2));
% coriolis matrix
C = -S.m2*S.l1*S.lc2*s2*[v(2), v(1) + v(2);
                    -v(1), 0] + diag([.2;.2]);
% mass elements
m11 = S.m1*S.lc1^2 + S.m2*(S.l1^2 + S.lc2^2 + 2*S.l1*S.lc2*c2) + ...
      S.I1 + S.I2;
m12 = S.m2*(S.lc2^2 + S.l1*S.lc2*c2) + S.I2;
m22 = S.m2*S.lc2^2 + S.I2;
% mass matrix
M = [m11, m12;
     m12, m22];
% gravity, damping, etc...
N = [(S.m1*S.lc1 + S.m2*S.l1)*S.g*c1 + S.m2*S.lc2*S.g*c12;
      S.m2*S.lc2*S.g*c12];
  
function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T
Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
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

