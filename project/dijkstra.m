function path = dijkstra(   start , goal , map)

    start_idx = pos2idx(map , start);
    goal_idx  = pos2idx(map , goal);
    start_idx =  reshape( start_idx, [1,3]);
    goal_idx  =  reshape( goal_idx , [1,3]);
    wdh = pos2idx(map , map.boundary(4:6))
    width = wdh(1);
    depth = wdh(2);
    height = wdh(3);
    pre = zeros(width, depth , height , 3 );
    dist = ones(width, depth , height) * inf;
    
    
    dist(start_idx(1), start_idx(2), start_idx(3)) = 0; % set start dist as 0

    visit = zeros(width, depth , height); % visited:1
    %6 directions
    dx = [1,-1,0,0,0,0];
    dy = [0,0,1,-1,0,0];
    dz = [0,0,0,0,1,-1];
    queue = zeros(1000000,3);
    head = 1;
    tail = 1;
    tail = tail+1;

    
     queue(tail,:) = start_idx;
     while(head<tail)
     
         head = head+1;
         now_idx = queue(head,:);
         %visit( now_idx)=0;
         visit( now_idx(1),now_idx(2),now_idx(3))=0;
         for k = 1:6
            new_x = now_idx(1)+ dx(k);
            new_y = now_idx(2)+ dy(k);
            new_z = now_idx(3)+ dz(k);
            if (new_x < 1 || new_x>width )
                continue;
            end
            if (new_y < 1 || new_y>depth )
                continue;
            end
            if (new_z < 1 || new_z>height )
                continue;
            end
            if (map.occgrid(new_x , new_y , new_z ) == 1 )
                continue;
            end
            if(dist(new_x , new_y , new_z ) > dist(now_idx(1),now_idx(2),now_idx(3)) + 1 )
            
                dist(new_x , new_y , new_z ) = dist(now_idx(1),now_idx(2),now_idx(3)) + 1;
                pre(new_x , new_y , new_z,: ) = [now_idx(1),now_idx(2),now_idx(3)];
                if( visit(new_x , new_y , new_z ) == 0)
                    tail = tail+1;
                    queue(tail,:) = [new_x , new_y , new_z];
                    visit(new_x , new_y , new_z ) = 1;
                end
            end
%             head
%             tail
         end
     end
dist
    
    path = [];
    length = 0;
    if ( dist(goal_idx(1), goal_idx(2), goal_idx(3) ) ) ~= inf
        % start from goal
        disp('orz');
        cur_p = goal_idx;
        while(1)
            cur_p =  reshape( cur_p , [1,3]);
            queue(end - length , :) = cur_p;
            length = length +1;
            if (cur_p == start_idx)
                break;
            end
            cur_p = pre(cur_p(1), cur_p(2), cur_p(3), :);
        end
        path = queue(end-length:end,:)
    end
    
% path = idx_to_points(map, path);
%if size(path, 1) > 0
% path = [start_xyz; path; goal_xyz];
% else
% path = zeros(0, 3);
% end
end