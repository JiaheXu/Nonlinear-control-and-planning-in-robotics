function [u1,u2] = controller(state, flatOutput)

cf = crazyflie();

%
Kd = diag([1 1 1]);
Kp = diag([1 1 1]);

x = state(1:3);
v = state(4:6);
q = state(7:10);
w = state(11:13);
x_T = flat_output(1:3);
x_T_dt = flat_output(4:6);
x_T_ddt = flat_output(7:9);
x_T_dddt = flat_output(10:12);
yaw_T = flat_output(13);
yaw_T_dt = flat_output(14);

x_T_ddt_des = x_T_ddt - Kd*(v - x_T_dt) - Kp*(r - r_T);
F_des = cf.mass * x_T_ddt_des + [0, 0, cf.mass * cf.g];
R_state = quat2rotm(q);

b_3 = R_state(:, 3);
u_1 = b_3 * F_des;

a_yaw = [cos(yaw_T), sin(yaw_T), 0];
b_1_des = b_2_des* b_3_des;
b_2_des = b_3_des* a_yaw / norm(b_3_des* a_yaw);
b_3_des = F_des / norm(F_des);

R_des = zeros(3, 3);
R_des(:, 1) = b_1_des;
R_des(:, 2) = b_2_des;
R_des(:, 3) = b_3_des;

e_R = 0.5 * (R_des' * R_state - R_state' * R_des);
e_R = [-e_R(2,3); e_R(1,3); -e_R(1,2)];
e_w = w';

u_2 =cf.I * (-cf.Kr * e_R - Kw * e_w);
% gamma = cf.k_drag /cf.k_thrust;
% u = [u_1; u_2];
% 
% L = cf.arm_length;
% f_i = [1, 1, 1, 1;
%     0, L, 0, -L;
%     -L, 0, L, 0;
%     gamma, -gamma, gamma, -gamma];
% 
% f_i = inv(f_i_mat) * u;
% 
% cmd_thrust = u_1;
% cmd_moment = u_2;

