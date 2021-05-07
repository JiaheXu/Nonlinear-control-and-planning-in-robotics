clc;
clear;

v = 0.2;
v2 = 0.25;
voxel = [v,v,v2];
map = load_map('C:\Users\Administrator\Desktop\project\maps\map0.txt', voxel );
map.occgrid;

path=[ 0         0         0
    0.8453    1.7957    0.3203
    0.9671    2.1200    0.1524
    1.1860    2.3008    0.1346
    3.1064    2.7020    0.3868
    4.4341    3.1994    0.8602
    5.3940    3.6919    0.7026
    5.8532    3.9465    0.9726
    6.3394    4.9187    0.8149
    7.0000    6.0000    1.0000];

subpath_check(map , path(1,:) ,path(5,:) )

