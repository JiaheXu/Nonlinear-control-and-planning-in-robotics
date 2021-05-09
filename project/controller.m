function [u1,u2] = controller(desired_state,current_state,n)
% desired_state: matrix of all the desited_state
% n: the current node in desired state 

cf = crazyflie();

%
Kd = [10;10;10];
Kp = [10;10;10];
Kd_2 = [10;10;10];
Kp_2 = [10;10;10];


% x = state(1:3);
% v = state(4:6);
% q = state(7:10);
% w = state(11:13);

x = current_state{n}.pos;
x_dt = current_state{n}.vel;
x_ddt = current_state{n}.acc;
yaw = current_state{n}.yaw;
yaw_dt = current_state{n}.yawdot;
angle = current_state{n}.angle;
omega = current_state{n}.omega;

% desired states from trajectory generator 
x_T = desired_state{n}.pos;
x_dt_T = desired_state{n}.vel;
x_ddt_T = desired_state{n}.acc;
yaw_T = desired_state{n}.yaw;
yaw_dt_T = desired_state{n}.yawdot;

%desired acceleration 
x_ddt_des = x_ddt_T - Kd.*(x_dt - x_dt_T) - Kp.*(x - x_T);

% Desired roll, pitch, yaw
angle1_des = 1/cf.g * (x_ddt_des(1)*sin(yaw_T) - x_ddt_des(2)*cos(yaw_T));
angle2_des = 1/cf.g * (x_ddt_des(1)*cos(yaw_T) + x_ddt_des(2)*sin(yaw_T));
angle3_des = yaw_T;

angle_des = [angle1_des;angle2_des;angle3_des];
omega_des = [0;0;yaw_dt_T];


u1 = cf.mass * x_ddt_des(3,1) + cf.mass * cf.g;

u2 = cf.I * (-Kd_2 * (omega - omega_des) - Kp_2 * (angle - angle_des));
