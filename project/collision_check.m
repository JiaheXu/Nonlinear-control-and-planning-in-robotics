function collision = collision_check(map, points)
%  Test whether points collide with any obstacle in the environment.
%  points is an n by 3 matrix which represent n points in a traj
%  collision = 1 if collision happens
    idx = pos2idx(map, points);
    collision = 0;
    if ( sum(map.occgrid(idx)) > 0 )
        collision = 1;
    end

end
