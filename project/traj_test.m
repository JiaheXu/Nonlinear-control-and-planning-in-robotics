clear all 
close all 
clc 

addpath(genpath('./'));
% voxel's width depth and height better to be the same value
v = 0.3;
v2 = 0.5;
voxel = [v,v,v2];
% map = load_map('C:\Users\Administrator\Desktop\project\maps\map1.txt', voxel );
map = load_map('C:\Users\rayzh\Documents\GitHub\Nonlinear-control-and-planning-in-robotics\project\maps\map1.txt', voxel );
% valid path1
pt = [
    0         0         0
    0.0833    0.0833    0.0833
    0.1667    0.1667    0.1667
    0.2500    0.2500    0.2500
    0.3333    0.3333    0.3333
    0.4167    0.4167    0.4167
    0.5000    0.5000    0.5000
    0.5000    0.8333    0.5000
    0.5000    1.1667    0.5000
    0.5000    1.5000    0.5000
    0.5000    1.8333    0.5000
    0.5000    2.1667    0.5000
    0.5000    2.5000    0.5000
    1.0000    2.5000    0.5000
    1.5000    2.5000    0.5000
    2.0000    2.5000    0.5000
    2.5000    2.5000    0.5000
    3.0000    2.5000    0.5000
    3.5000    2.5000    0.5000
    3.5000    2.6667    0.5000
    3.5000    2.8333    0.5000
    3.5000    3.0000    0.5000
    3.5000    3.1667    0.5000
    3.5000    3.3333    0.5000
    3.5000    3.5000    0.5000
    4.0000    3.5000    0.5000
    4.5000    3.5000    0.5000
    5.0000    3.5000    0.5000
    5.5000    3.5000    0.5000
    6.0000    3.5000    0.5000
    6.5000    3.5000    0.5000
    6.5000    3.8333    0.5000
    6.5000    4.1667    0.5000
    6.5000    4.5000    0.5000
    6.5000    4.8333    0.5000
    6.5000    5.1667    0.5000
    6.5000    5.5000    0.5000
    6.5833    5.5833    0.5833
    6.6667    5.6667    0.6667
    6.7500    5.7500    0.7500
    6.8333    5.8333    0.8333
    6.9167    5.9167    0.9167
    7.0000    6.0000    1.0000];
% valid path2
path1 = [ 0 0 0;
    0.5 0.5 0.5;
    0.5 2.5 0.5;
    3.5 2.5 0.5;
    3.5 3.5 0.5;
    6.5 3.5 0.5;
    6.5 5.5 0.5;
    7 6 1;
    ];
plot3(pt(:,1),pt(:,2),pt(:,3), '-r')
hold on
time = 0;
tstep = 0.01;
ind = 1;

x = zeros(1000,1);
y = zeros(1000,1);
z = zeros(1000,1);
[sol, timeVec ,timedtVec ]= minimum_jerk(pt);
state_num = timeVec(end,1)/tstep+1;
start = [0;0;0];
state0 = zeros(12,1);
state0(1:3,1) = start;
sn = 1;
t_interval = 0:tstep:timeVec(end,1);

for i = 0:tstep:timeVec(end,1)
    state{sn}    = state0;
    %     xtraj{sn} = zeros(state_num, 12);
    %     ttraj{sn} = zeros(state_num, 1);
    sn = sn+1;
end

for t = 0:tstep:timeVec(end,1)
    if ind<length(t_interval)
        %for t = 1:1
        desired_state = trajectory_generator(t , pt , sol , timeVec ,timedtVec );
        x(ind) = desired_state.pos(1,1);
        y(ind) = desired_state.pos(2,1);
        z(ind) = desired_state.pos(3,1);
        ind = ind+1;
        
        
        %     x{qn} = xsave(end, :)';
        % Save to traj
        %     xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        %     ttraj{qn}((iter-1)*nstep+1:iter*nstep)   = tsave(1:end-1);
    end
    
end
plot3(x,y,z);

T = sum(timedtVec);
%initial state
x = zeros(12,1);

S.sol = sol;
S.timeVec = timeVec;
S.timedtVec = timedtVec;
S.pt = pt;

[ts, xs] = ode45(@uni_ode , [0 T], x, [], S);

% figure(2)
plot3(xs(:,1),xs(:,2),xs(:,3))



function state_dot = uni_ode(t, state , S)
    desired_state = trajectory_generator(t , S.pt , S.sol , S.timeVec ,S.timedtVec );
    
    [u1,u2] = controller(desired_state , state);
    state_dot = EOM(state , u1,u2);
end

