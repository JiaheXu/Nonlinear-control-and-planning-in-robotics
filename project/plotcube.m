function plotcube(map)

    for i = 1:size(map.blocks , 1)
        edges = map.blocks(i ,4:6) - map.blocks(i ,1:3);
        origin = map.blocks(i ,1:3);
        plot_obstacle(edges,origin,1,[1 0 0])
        view(3);
    end
%plot_obstacle([5 5 5],[ 2  2  2],1,[1 0 0]);
%plot_obstacle([5 5 5],[10 10 10],1,[1 0 0]);
%plot_obstacle([5 5 5],[20 20 20],1,[1 0 0]);
function plot_obstacle(varargin)
% PLOTCUBE - Display a 3D-cube in the current axes
%
%   PLOTCUBE(EDGES,ORIGIN,ALPHA,COLOR) displays a 3D-cube in the current axes
%   with the following properties:
%   * EDGES : 3-elements vector that defines the length of cube edges
%   * ORIGIN: 3-elements vector that defines the start point of the cube
%   * ALPHA : scalar that defines the transparency of the cube faces (from 0
%             to 1)
%   * COLOR : 3-elements vector that defines the faces color of the cube
%
inArgs = varargin;

[edges,origin,clearance,color] = deal(inArgs{:});

XYZ = {
  [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ;
  [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ;
  [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ;
  [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ;
  [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ;
  [0 1 1 0]  [0 0 1 1]  [1 1 1 1] ;
  };
% mat2cell(edges,1,[1 1 1])
% mat2cell(origin,1,[1 1 1])
XYZ = mat2cell(...
cellfun( @(x,y,z) x*y+z ,XYZ ,...
repmat(mat2cell(edges,1,[1 1 1]),6,1) ,...
repmat(mat2cell(origin,1,[1 1 1]),6,1) ,...
'UniformOutput',false), ...
6,[1 1 1]);

% XYZ = mat2cell(...
% cellfun( @(x,y,z) x*y+z ,XYZ ,...
% repmat(mat2cell(edges,1,[1 1 1]),6,1) ,...
% repmat(mat2cell(origin,1,[1 1 1]),6,1) ,...
% 'UniformOutput',false), ...
% 6,[1 1 1]);


cellfun(@patch,XYZ{1},XYZ{2},XYZ{3}, repmat({color},6,1),repmat({'FaceAlpha'},6,1),repmat({clearance},6,1));

