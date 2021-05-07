function path = add_point(start,goal,points)

    path = zeros(points,3);
    for j = 1:3
        path( 1 : points, j ) = linspace(start(1,j), goal(1,j), points);
    end
end