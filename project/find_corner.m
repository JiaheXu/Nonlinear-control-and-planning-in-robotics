function corner = find_corner( path )
% clc; 
% clear;
% path = [
%     0         0         0
%     0.5000    0.5000    0.5000
%     0.5000    1.5000    0.5000
%     0.5000    2.5000    0.5000
%     1.5000    2.5000    0.5000
%     2.5000    2.5000    0.5000
%     3.5000    2.5000    0.5000
%     3.5000    3.5000    0.5000
%     4.5000    3.5000    0.5000
%     5.5000    3.5000    0.5000
%     6.5000    3.5000    0.5000
%     6.5000    4.5000    0.5000
%     6.5000    5.5000    0.5000
%     7.0000    6.0000    1.0000];

len = size( path , 1);
corner = zeros(100000,3);
length = 1;
corner(1,:) = path(1,:);
for i = 2:len-1
%     path(i,:) - path(i-1,:)
%     path(i+1,:)-path(i,:)
    delta = path(i,:) - path(i-1,:) - path(i+1,:) + path(i,:);
    if( norm(delta) > 0.1 )
        %path(i,:) - path(i-1,:) ~= path(i+1,:)-path(i,:)
        length = length+1;
        corner(length,:) = path(i,:);
    end
end
length = length+1;
corner(length,:) = path(len,:);
corner = corner(1:length,:)
length