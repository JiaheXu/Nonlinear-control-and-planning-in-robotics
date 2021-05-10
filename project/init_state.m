function [ s ] = init_state( start, yaw )
%Initialize 12 x 1 state vector

s     = zeros(13,1);

s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = 0;        %xdot
s(5)  = 0;        %ydot
s(6)  = 0;        %zdot
s(7)  = 0;        %roll
s(8)  = 0;        %pitch
s(9)  = yaw;      %yaw
s(10) = 0;        %p
s(11) = 0;        %q
s(12) = 0;        %r

end