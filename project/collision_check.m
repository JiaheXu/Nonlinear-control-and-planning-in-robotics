function collision = collision_check(map, points)
%  Test whether points collide with any obstacle in the environment.
%  points is an n by 3 matrix which represent n points in a traj
%  collision = 1 if collision happens
idx = pos2idx(map, points);
collision = 0;
if ( sum(map.occgrid(idx)) > 0 )
    collision = 1;
end
% idx = (points(:,3)-1)*nx*ny + (points(:,2)-1)*nx + points(:,1);
% C = map3d(idx) ~= 255 | map3d(nx*ny*nz+idx) ~= 255 | map3d(nx*ny*nz*2+idx) ~= 255;

end
