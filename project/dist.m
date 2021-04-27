 function cost = L2dist(start_pos, goal_pos)
    cost = sqrt( sum((double(start_pos) - double(goal_pos)).^2 ) );
 end