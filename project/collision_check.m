function collision = collision_check(map, points)
%  Test whether points collide with any obstacle in the environment.
%  points is an Nx 3 matrix which represent n points in a traj
%  collision = 1 if collision happens
    %valid = 0;
    idx = pos2idx(map, points);
    
    collision = 0;
%     size(map.occgrid)
%      map.occgrid
%      map.occgrid(2,2,1)
%     idx(1:end,1:end)
%     sum( map.occgrid(idx(:,:)) )
%     tmp = sum(map.occgrid(idx(1:end,1:end)))
% %     tmp = max(tmp)
    if(min(points(:,1)) < map.boundary(1)|| max(points(:,1))> map.boundary(4) )
        collision = 1;
    end
    if(min(points(:,2)) < map.boundary(2) || max(points(:,2))> map.boundary(5) )
        collision = 1;
    end
    if(min(points(:,3)) < map.boundary(3) || max(points(:,3))> map.boundary(6) )
        collision = 1;
    end
    collision;
    if(collision == 0)
        for i = 1:size(points , 1)
            if(map.occgrid(idx(i,1),idx(i,2),idx(i,3))==1)
                collision = 1;
            end
        end    
    end
end
