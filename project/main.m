clc;
clear;
close all;
addpath(genpath('./'));

% test for load_map
disp('loading map');
% voxel's width depth and height better to be the same value
v = 0.1;
v2 = 0.2;
voxel = [v,v,v2];
% map = load_map('C:\Users\Administrator\Desktop\project\maps\map3.txt', voxel );
map = load_map('C:\Users\rayzh\Documents\GitHub\Nonlinear-control-and-planning-in-robotics\project\maps\map0.txt', voxel );

map.occgrid;
plotcube(map)
% 
start = map.boundary(1:3)
goal = map.boundary(4:6)

path = dijkstra( map , start, goal);

plot3(path(:,1), path(:,2), path(:,3), '-b');
hold on
path = find_corner(path);
path = CEM( map , path , start, goal)

plot3(path(:,1), path(:,2), path(:,3), '-g');
hold on 
% path =[
%          0         0         0
%     0.5913    0.6475    0.4486
%     1.0189    2.1353    0.5779
%     2.5732    2.5652    1.2788
%     3.5257    2.9342    1.7865
%     3.9401    3.2465    2.2173
%     4.9250    3.7122    2.8868
%     5.8049    3.9219    3.0740
%     6.5386    5.4535    4.4235
%     7.0000    6.0000    5.0000];
% 
% path = build_line(path(8:9 , : ) ,800);
% collision_check( map , path)
% path = build_line(path,10);
% plot3(path(:,1), path(:,2), path(:,3), '-g');
% hold on ;
plotcube(map);

% time = 0;
% tstep = 0.01;
% ind = 1;
% 
% x = zeros(1000,1);
% y = zeros(1000,1);
% z = zeros(1000,1);
% [sol, timeVec ,timedtVec ]= minimum_jerk(path);
% 
% for t = 0:tstep:timeVec(end,1)
% %for t = 1:1
%     desired_state = trajectory_generator(t , path , sol , timeVec ,timedtVec );
%      x(ind) = desired_state.pos(1,1);
%      y(ind) = desired_state.pos(2,1);
%      z(ind) = desired_state.pos(3,1);
%     ind = ind+1; 
% end
% plot3(x,y,z,'-y');
% hold on;
% plotcube(map);
% time_tol = 30;          % maximum simulation time
% starttime = 0;          % start of simulation in seconds
% 
% tstep     = 0.01;       % this determines the time step at which the solution is given
% cstep     = 0.05;       % image capture time interval
% nstep     = cstep/tstep;
% time      = starttime;  % current time
% max_iter  = time_tol / cstep;      % max iteration


%path = CEM( map , start, goal)

%path3 = CEM( map , start, goal);
%collision_check( map , path3)

%plot3(path3(:,1), path3(:,2), path3(:,3), '-b');
% todo 
% combine the result of dijkstra and CEM
% more robust collision checking constant step or algebric methods
idxpath = [1     1     1
     2     1     1
     2     2     1
     2     3     1
     2     4     1
     2     5     1
     3     5     1
     4     5     1
     5     5     1
     6     5     1
     7     5     1
     8     5     1
     8     6     1
     8     7     1
     9     7     1
    10     7     1
    11     7     1
    12     7     1
    13     7     1
    14     7     1
    14     8     1
    14     9     1
    14    10     1
    14    11     1
    14    12     1
    14    12     2];
idx2pos(map , idxpath);

% valid path1
path1 = [
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
path2 = [ 0 0 0;
       0.5 0.5 0.5;
       0.5 2.5 0.5;
       3.5 2.5 0.5;
       3.5 3.5 0.5;
       6.5 3.5 0.5;
       6.5 5.5 0.5;
       7 6 1;
    ];

pt = path; 
time = 0;
tstep = 0.01;
ind = 1;

x = zeros(1000,1);
y = zeros(1000,1);
z = zeros(1000,1);
[sol, timeVec ,timedtVec ]= minimum_jerk(pt);
state_num = timeVec(end,1)/tstep+1;
state0 = zeros(12,1);
state0(1:3,1) = start;
sn = 1;
t_interval = 0:tstep:timeVec(end,1);

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
x(1:3) = start;
S.sol = sol;
S.timeVec = timeVec;
S.timedtVec = timedtVec;
S.pt = pt;

[ts, xs] = ode45(@uni_ode , [0 T], x, [], S);

% figure(2)
plot3(xs(:,1),xs(:,2),xs(:,3))
x(1:3) = start';

figure(2)
plot3(xs(:,1),xs(:,2),xs(:,3))

function state_dot = uni_ode(t, state , S)
    desired_state = trajectory_generator(t , S.pt , S.sol , S.timeVec ,S.timedtVec );
    
    [u1,u2] = controller(desired_state , state);
    state_dot = EOM(state , u1,u2);
end


