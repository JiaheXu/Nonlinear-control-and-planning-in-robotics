%map = lad_map('C:\Users\Administrator\Desktop\quadrotor-master\traj_planning\maps\map1.txt', [0.1, 0.1, 2])
function map = load_map(filename, res_xyz)
% MAP = LOAD_MAP(filename, res_xy, res_z, margin) loads the environment
%   res_xyz   - 1x3 vector of the length (in meters) of each side of the each voxel in the grid
%   boundary - 1x6 vector of the minimum and maximum corners of 
%   the bounding box of the environment stored as [x_min y_min z_min x_max y_max z_max]
%   blocks    - 1x9 vector of the minimum (lower-left-front) and maximum
%               (upper-right-back) corners of each cuboid obstacle along
%               with a RGB color triplet for visualization stored as:
%               [x_min y_min z_min x_max y_max z_max R G B]
%   margin    - the distance (in meters) from an obstacle at which a path
%               is considered to be in collision (NOTE: each obstacle is
%               inflated by margin on all sides before intersecting voxels
%               are marked as occupied in the grid)
%   occgrid   - a 3D matrix of voxels with value 0 if the area enclosed by
%               the voxel is free and value 1 if an obstacle intersects the
%               voxel (NOTE: the origin of each voxel is located at the
%               lower-left-front corner)
%
% parameters:
%   filename - text file containing map bounds and obstacles
%   res_xy   - grid resolution in x, y dimensions in meters
%   res_z    - grid resolution in z dimension in meters
%   margin   - the distance with which to inflate obstacles in the
%              resulting map to avoid collisions

fid = fopen(filename);
lines = textscan(fid, '%s %f %f %f %f %f %f %f %f %f');
fclose(fid);

[n,m] = size(lines{1});
vals = [lines{2:9}];
types = lines{1};
boundary = [];
blocks = [];
for i = 1 : n
    type = types{i};
    if strcmp(type, 'boundary') == 1
        boundary = vals(i, :);
    end
    if strcmp(type, 'block') == 1
        blocks = [blocks; vals(i, :)];
    end
end

map.res_xyz = res_xyz;
map.boundary = boundary;
map.blocks = blocks;

% construct voxel grid
% width depth height 
wdh =  floor((boundary(4:6)-boundary(1:3))./map.res_xyz)
width = wdh(1);
depth= wdh(2);
height = 1;
if(size(wdh,2)>2)
    height = wdh(3);
end

map.width = width;
map.depth = depth;
map.height = height;

map.occgrid = reshape( zeros(wdh),[width, depth, height]);

[m , n] = size(blocks);
%valid = 1;
for i = 1:m
  ijk_min = pos2idx(map, blocks(i,1:3));
  ijk_max = pos2idx(map, blocks(i,4:6) - res_xyz) ;
  map.occgrid(ijk_min(1):ijk_max(1), ijk_min(2):ijk_max(2),ijk_min(3):ijk_max(3)) = 1;
end

end

