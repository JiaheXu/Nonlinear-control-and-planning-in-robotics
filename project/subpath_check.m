function f = subpath_check(map,start,goal)
    points = 100;
    path = zeros(points,3);
    f = 0;

    for j = 1:3
        path( 1 : points, j ) = linspace(start(1,j), goal(1,j), points);
    end
    %#########################################

    if( collision_check(map, path) == 1 )
        f = 1;
    end
end