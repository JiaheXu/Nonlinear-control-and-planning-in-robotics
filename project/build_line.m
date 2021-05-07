function lines = build_line( path , points )
    n = size(path,1);
    lines = zeros( (n - 1)*points + 1 , 3 );
    for i=1:n-1
        for j = 1:3
            lines( (i-1)*points+1 : i*points + 1 , j ) = linspace(path(i,j), path(i+1,j), points+1);
        end
    end
end