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

map = load_map('C:\Users\Administrator\Desktop\project\maps\map3.txt', voxel );

map.occgrid;
plotcube(map)
start = map.boundary(1:3)
goal = map.boundary(4:6)

path = dijkstra( map , start, goal);

plot3(path(:,1), path(:,2), path(:,3), '-y');
hold on
path = find_corner(path);

iter = 20;
[path , crs ,Time] = CEM( map , path , start, goal,iter);
path = build_line(path ,20);
plot3(path(:,1), path(:,2), path(:,3), '-b');
hold on 

plotcube(map);
hold on

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
        x_des(ind) = desired_state.pos(1,1);
        y_des(ind) = desired_state.pos(2,1);
        z_des(ind) = desired_state.pos(3,1);
        ind = ind+1;
    end
    
end
plot3(x_des,y_des,z_des)
hold on

T = sum(timedtVec);
%initial state
x = zeros(12,1);
x(1:3) = start;
S.sol = sol;
S.timeVec = timeVec;
S.timedtVec = timedtVec;
S.pt = pt;

[ts, xs] = ode45(@uni_ode , [0 T], x, [], S);
plot3(xs(:,1),xs(:,2),xs(:,3),'-r')


function state_dot = uni_ode(t, state , S)
    desired_state = trajectory_generator(t , S.pt , S.sol , S.timeVec ,S.timedtVec );
    
    [u1,u2] = controller(desired_state , state);
    state_dot = EOM(state , u1,u2);
end


