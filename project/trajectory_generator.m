function [ desired_state ] = trajectory_generator(t,path)

% input----
% t: current time 
% path: way points of the path 3xn matrix 
% output---
% desired state containing position, velocity, acceleration, jerk, snap,
% yaw, and yaw angular velocity 


n = size(path,2); % number of waypoints
speed = 2; % speed of the hover, can be tuned

time = 0;
timeVec = zeros(1,n);
timedtVec = zeros(1,n);


for i = 2:n
    dist = norm(path(:,i) - path(:,i-1));
    timedtVec(i) = dist/speed;
    time = time + timedtVec(i);
    timeVec(i) = time;
end
total_time = sum(timedtVec);

% initial boundary
A = zeros(6 * (n - 1),6 * (n - 1));
b = zeros(6 * (n - 1), 3);
A(1:3, 1:6) =  [0, 0, 0, 0, 0, 1;
                0, 0, 0, 0, 1, 0;
                0, 0, 0, 2, 0, 0];
b(1,:) = path(1, :);
% end boundary
dt = timedtVec(end);
A(end-2:end, end-5:end) = [dt^5,    dt^4,   dt^3,   dt^2,  dt, 1;
                           5*dt^4,  4*dt^3, 3*dt^2, 2*dt,  1,  0;
                           20*dt^3, 12*dt^2,6*dt,   2,     0,  0];
b(end-2, :) = path(end,:);

for i = 1:n-2
    dt = timedtVec[i];       
    A(6*i-2:6*i+4, 6*i-5:6*i+7) =  [dt^5,   dt^4, dt^3, dt^2,dt, 1,   0,0,0,0,0,0;
                                    0,0,0,0,0,0,                      0,0,0,0,0,1;
                                    5*dt^4,  4*dt^3,3*dt^2,2*dt,1,0,  0,0,0,0,-1,0;
                                    20*dt^3, 12*dt^2,6*dt,2,0,0,      0,0,0,-2,0,0;
                                    60*dt^2,  24*dt,6,0,0,0,          0,0,-6,0,0,0;
                                    120*dt^2, 24,0,0,0,0,             0,-24,0,0,0,0];
    
    b(6*i-2, :) = path(i, :);
    b(6*i-1, :) = path(i, :);
    
end
solution = A\b;    
   
if t >= total_time   % if there is only on point in the path 
    pos = path(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
    jerk =[0;0;0];
    snap =[0;0;0];
else
       
    % 5th order minimum jerk trajectory
    k = find(ts<=t);
    k = k(end);
    dt = t-timeVec(k-1);
    coeff = solution(6*(k-1)+1:6*k,:);
    pos = [dt^5,     dt^4,    dt^3,    dt^2,    dt,  1]*coeff;
    vel = [5*dt^4,   4*dt^3,  3*dt^2,  2*dt,    1,  0]*coeff;
    acc = [20*dt^3,  12*dt^2, 6*dt,    2,      0,  0]*coeff;
    jerk= [60*dt^2, 24*dt,   6,       0,      0,  0]*coeff;
    snap= [120*dt,  24,     0,       0,      0,  0]*coeff;
   
end

yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.jerk = jerk(:);
desired_state.snap = snap(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
