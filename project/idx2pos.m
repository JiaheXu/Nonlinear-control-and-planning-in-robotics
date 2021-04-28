function xyz = idx2pos(map, ijk)
% convert index in the center of occupancid grids xyz (Nx3) 
xyz = ones(size(ijk, 1), 3);
    if size(ijk,1) > 0
        xyz(:,1) = map.boundary(1)+(ijk(:,1)-1)*map.res_xyz(1) + map.res_xyz(1)*0.5;
        xyz(:,2) = map.boundary(2)+(ijk(:,2)-1)*map.res_xyz(2) + map.res_xyz(2)*0.5;
        xyz(:,3) = map.boundary(3)+(ijk(:,3)-1)*map.res_xyz(3) + map.res_xyz(3)*0.5;
    else
        xyz = [];
    end
end