clc;
clear;
close all;
addpath(genpath('./'));

% test for load_map
disp('loading map');
% voxel's width depth and height better to be the same value
v = 0.3;
v2 = 0.5;
voxel = [v,v,v2];
map = load_map('C:\Users\Administrator\Desktop\project\maps\map1.txt', voxel );
map.occgrid;
plotcube(map)

start = map.boundary(1:3);
goal = map.boundary(4:6);

collision_check( map , start);
collision_check( map , goal);

path = dijkstra( map , start, goal);
collision_check( map , path);

plot3(path(:,1), path(:,2), path(:,3), '-b');
path = find_corner(path);
path = CEM( map , path , start, goal);
plot3(path(:,1), path(:,2), path(:,3), '-g');
plotcube(map)

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

