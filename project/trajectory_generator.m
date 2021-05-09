function [ desired_state ] = trajectory_generator(t,pt,solution , timeVec , timedtVec)
total_time = timeVec(end,1);

if t >= total_time   % if there is only on point in the path 
    
    pos = pt(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
    jerk =[0;0;0];
    snap =[0;0;0];
else
       
    % 5th order minimum jerk trajectory
%     k = find(timeVec<=t);
    k = 1;
    for ii = 1:length(timeVec)
        if timeVec(ii)<=t
            k = ii;
        end
    end
    
%     k = k(end);
    dt = t-timeVec(k);
    %origin
    %coeff = solution(6*k+1:6*k+6,:);
    k;
    coeff = solution(6*k-5 : 6*k,:);
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

% if collision_check(map,desired_state.pos)
%     desired_state = 0;  % if collision happens, return 0 and replan the path 
% end 