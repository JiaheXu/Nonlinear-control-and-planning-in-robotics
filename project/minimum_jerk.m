function [solution ,timeVec, timedtVec ] = minimum_jerk(path)
% path = [ 0 0 0;
%        0.5 0.5 0.5;
%        0.5 2.5 0.5;
% %        3.5 2.5 0.5;
% %        3.5 3.5 0.5;
% %        6.5 3.5 0.5;
% %        6.5 5.5 0.5;
% %        7 6 1;
%     ];

% input----
% t: current time 
% path: way points of the path nx3n matrix 
% output---
% desired state containing position, velocity, acceleration, jerk, snap,
% yaw, and yaw angular velocity 

n = size(path,1); % number of waypoints
speed = 1; % speed of the hover, can be tuned

time = 0;
% timeVec(i) is the start time of i-th segment( from point i-1 to i)
timeVec = zeros(n,1);

timedtVec = zeros(n,1);


for i = 2:n
    dist = norm(path(i,:) - path(i-1,:));
    timedtVec(i) = dist/speed;
    time = time + timedtVec(i);
    timeVec(i) = time;
end
total_time = sum(timedtVec)

% % initial boundary
A = zeros(6 * (n - 1),6 * (n - 1));
b = zeros(6 * (n - 1), 3);
A(1:3, 1:6) =  [0, 0, 0, 0, 0, 1;
                0, 0, 0, 0, 1, 0;
                0, 0, 0, 2, 0, 0];
b(1,:) = path(1, :);
% % end boundary
dt = timedtVec(end);
%A(end-2:end, end-5:end)
A(end-2:end, end-5:end) = [dt^5,    dt^4,   dt^3,   dt^2,  dt, 1;
                           5*dt^4,  4*dt^3, 3*dt^2, 2*dt,  1,  0;
                           20*dt^3, 12*dt^2,6*dt,   2,     0,  0];
b(end-2, :) = path(end,:);
% 
for i = 1:n-2
    dt = timedtVec(i+1);       
    A( 6*i-2 : 6*i+3, 6*i-5 : 6*i+6 ) =  [dt^5,   dt^4, dt^3, dt^2,dt, 1,     0,0,0,0,0,0;
                                    0,0,0,0,0,0,                              0,0,0,0,0,1;
                                    5*dt^4,  4*dt^3 , 3*dt^2 , 2*dt ,1,0,     0,0,0,0,-1,0;
                                    20*dt^3, 12*dt^2,6*dt,2,0,0,              0,0,0,-2,0,0;
                                    60*dt^2,  24*dt,6,0,0,0,                  0,0,-6,0,0,0;
                                    %# ???
                                    120*dt, 24,0,0,0,0,                       0,-24,0,0,0,0];
    
    b(6*i-2, :) = path(i+1, :);
    b(6*i-1, :) = path(i+1, :);
    
end
solution = A\b;