function ijk = pos2idx(map, pos)
ijk = floor( (pos- map.bound_xyz(1:3) )./map.res_xyz) +[1,1,1];
end