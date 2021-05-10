function [u1,u2] = controller(desired_state,current_state)
% desired_state: matrix of all the desited_state
% n: the current node in desired state 

cf = crazyflie();

% 
% Kd = [0.2;0.5;1.6];
% Kd = diag(Kd);
% Kp = [0.32;0.15;1.5];
% Kp = diag(Kp);
% Kd_2 = [1.2;1.5;4];
% Kd_2 = diag(Kd_2);
% Kp_2 = [1.3;1.0;4];
% Kp_2 = diag(Kp_2);

% Kd = [0.8;0.4;6];
% Kd = diag(Kd);
% Kp = [0.3;0.15;4.0];
% Kp = diag(Kp);
% Kd_2 = [1.3;2;3];
% Kd_2 = diag(Kd_2);
% Kp_2 = [1.4;1.5;5];
% Kp_2 = diag(Kp_2);

Kd = [2;0.8;2];
Kd = diag(Kd);
Kp = [0.05;0.05;2.0];
Kp = diag(Kp);
Kd_2 = [1.5;3;3];
Kd_2 = diag(Kd_2);
Kp_2 = [1.5;5;1];
Kp_2 = diag(Kp_2);

x = current_state(1:3,1);
x_dt = current_state(4:6,1);
angle = current_state(7:9,1);
omega = current_state(10:12,1);

% desired states from trajectory generator 
x_T = desired_state.pos;
x_dt_T = desired_state.vel;
x_ddt_T = desired_state.acc;
yaw_T = desired_state.yaw;
yaw_dt_T = desired_state.yawdot;

%desired acceleration 
% fprintf('x_ddt_T\n')
% size(x_ddt_T)
% 
% fprintf('x_dt - x_dt_T\n')
% size(x_dt - x_dt_T)
% 
% fprintf('x -----\n')
% size(x)
% 
% fprintf('x_T------\n')
% size(x_T)
% 
% fprintf('x - x_T\n')
% size( x - x_T )
% 
% fprintf('Kd*(x_dt - x_dt_T)\n')
% size( Kd*(x_dt - x_dt_T) )


% 
% fprintf('Kp*(x - x_T)\n')
% size(Kp*(x - x_T))
x_ddt_des = x_ddt_T - Kd*(x_dt - x_dt_T) - Kp*(x - x_T);
% fprintf('%d   %d   %d   %d   %d   %d \n',size(x_ddt_T,1),size(x_ddt_T,2),size(x_dt,1),size(x_dt,2),size(x_dt_T,1),size(x_dt_T,2));

% Desired roll, pitch, yaw
angle1_des = 1/cf.g * (x_ddt_des(1)*sin(yaw_T) - x_ddt_des(2)*cos(yaw_T));
angle2_des = 1/cf.g * (x_ddt_des(1)*cos(yaw_T) + x_ddt_des(2)*sin(yaw_T));
angle3_des = yaw_T;

angle_des = [angle1_des;angle2_des;angle3_des];
omega_des = [0;0;yaw_dt_T];


u1 = cf.mass * x_ddt_des(3,1) + cf.mass * cf.g;

u2 = cf.I * (-Kd_2 * (omega - omega_des) - Kp_2 * (angle - angle_des));
