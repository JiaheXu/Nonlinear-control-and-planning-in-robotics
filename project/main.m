clc;
clear;
close all;
addpath(genpath('./'));

% test for load_map
disp('loading map');
% voxel's width depth and height better to be the same value
v = 0.1;

% 0.3 works just fine
v2 = 0.3;
voxel = [v,v,v2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% you need to change the path to map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map = load_map('C:\Users\Administrator\Desktop\project\maps\map0.txt', voxel );

map.occgrid;

start = map.boundary(1:3)
goal = map.boundary(4:6)

path = dijkstra( map , start, goal);
dijkstra_path = path;


plot3(dijkstra_path(:,1), dijkstra_path(:,2), dijkstra_path(:,3),'y');
%plotcube(map);
%path(2:end-1 , 2) = path(2:end-1 , 2)/2


path = find_corner(path);
%#####################################################################

%%%%% for map3.txt iter needs to be larger
iter = 20;
sigma = .5;
[CEM_path , crs ,Time] = CEM( map , path , start, goal,iter,sigma);
path = build_line(CEM_path ,50);

figure(1);
rounds = size(crs,2)
xline = linspace(1, rounds , rounds);
plot(xline , crs , "-r")
legend("Distance of every round")

figure(2)
plot(xline , Time , "-b")
legend('Run Time of every round')

pt = path; 
time = 0;
tstep = 0.01;
ind = 1;

x_des = zeros(1000,1);
y_des = zeros(1000,1);
z_des = zeros(1000,1);
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


T = sum(timedtVec);
%initial state
x = zeros(12,1);
x(1:3) = start + [0.1,0.1,0.1];
S.sol = sol;
S.timeVec = timeVec;
S.timedtVec = timedtVec;
S.pt = pt;

[ts, xs] = ode45(@uni_ode , [0 T], x, [], S);

%%%%%%%%%%%%% ouput results into pictures
figure(3);
plot3(dijkstra_path(:,1), dijkstra_path(:,2), dijkstra_path(:,3),'y');
hold on
plot3(x_des,y_des,z_des, '-b')
hold on
plot3(xs(:,1),xs(:,2),xs(:,3),'-r')
legend('dijkstra trajectory' ,'desired trajectory', 'executed trajectory')
hold on
legend('AutoUpdate','off')
plotcube(map);


%% trajectory animation 
make_video = 0;
if make_video==1
    v = VideoWriter('quadPlanner.avi');
    figure(3)
    hold on
    % v.FrameRate = 1;
    open(v);
    for i = 1:20:size(xs,1)-20
        p = plot3(xs(i:i+19,1),xs(i:i+19,2),xs(i:i+19,3),'-r')
        p.LineWidth = 3;
        pause(0.5)
        im = frame2im(getframe(gcf));
        writeVideo(v,im);
    end
    close(v)
end


function state_dot = uni_ode(t, state , S)
    desired_state = trajectory_generator(t , S.pt , S.sol , S.timeVec ,S.timedtVec );
    
    [u1,u2] = controller(desired_state , state);
    % u1 u2 are small noise need to be smaller
    u1 = u1 + rand(1,1)*1e-8;
    u2 = u2 + rand(1,1)*1e-8;
    state_dot = EOM(state , u1,u2);
end


