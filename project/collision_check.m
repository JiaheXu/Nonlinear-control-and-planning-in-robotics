function collision = collision_check(map, path)
%  Test whether path collide with any obstacle in the environment.
%  path is an Nx 3 matrix which represent n path in a traj
%  collision = 1 if collision happens
    %valid = 0;
%     idx = pos2idx(map, path);
%     
    collision = 0;
%     size(map.occgrid)
%      map.occgrid
%      map.occgrid(2,2,1)
%     idx(1:end,1:end)
%     sum( map.occgrid(idx(:,:)) )
%     tmp = sum(map.occgrid(idx(1:end,1:end)))
% %     tmp = max(tmp)
    if(min(path(:,1)) < map.boundary(1)|| max(path(:,1))> map.boundary(4) )
        collision = 1;
    end
    if(min(path(:,2)) < map.boundary(2) || max(path(:,2))> map.boundary(5) )
        collision = 1;
    end
    if(min(path(:,3)) < map.boundary(3) || max(path(:,3))> map.boundary(6) )
        collision = 1;
    end
%     collision = 0;
%     if(collision == 0)
%         for i = 1:size(path , 1)
%             if(map.occgrid(idx(i,1),idx(i,2),idx(i,3))==1)
%                 collision = 1;
%             end
%         end    
%     end
    if(collision == 1)
        return
    end
    m = size(path,1);
    n = size(map.blocks,1);
    for i = 1 : m 
        for j = 1 : n
            x = path(i,1);
            y = path(i,2);
            z = path(i,3);
            
            if( map.blocks(j,1)<=x && map.blocks(j,4)>=x)
                if( map.blocks(j,2)<=y && map.blocks(j,5)>=y)
                    if( map.blocks(j,3)<=z && map.blocks(j,6)>=z)
                        collision = 1;
                        break;
                    end
                end
            end
        end
    end
    
end
