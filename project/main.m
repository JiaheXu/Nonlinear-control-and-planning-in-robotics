clc;
clear;
close all;
addpath(genpath('./'));

% test for load_map
disp('loading map');
% voxel's width depth and height better to be the same value
v = 1;
voxel = [v,v,v];
map = load_map('C:\Users\Administrator\Desktop\project\maps\map0.txt', voxel );
map.occgrid;
start = [0,0,0];
goal = [7,6,1];
% map.occgrid
path = dijkstra( start, goal , map);
collision_check( map , path);




