function ijk = pos2idx(map, pos)
    ijk = ones(size(pos,1),3);
    if size(pos,1) > 0
    ijk(:,1) = min(floor((pos(:,1) - map.boundary(1) )/map.res_xyz(1)) + 1 , map.width);
    ijk(:,2) = min(floor((pos(:,2) - map.boundary(2) )/map.res_xyz(2)) + 1 , map.depth);
    ijk(:,3) = min(floor((pos(:,3) - map.boundary(3) )/map.res_xyz(3)) + 1 , map.height);
    end
    
end