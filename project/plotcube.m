function plotcube(map)

    for i = 1:size(map.blocks , 1)
        edges = map.blocks(i ,4:6) - map.blocks(i ,1:3);
        origin = map.blocks(i ,1:3);
        plot_obstacle(edges,origin,1,[0 1 0])
        %view(3);
    end

function plot_obstacle(varargin)
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

XYZ = mat2cell(...
cellfun( @(x,y,z) x*y+z ,XYZ ,...
repmat(mat2cell(edges,1,[1 1 1]),6,1) ,...
repmat(mat2cell(origin,1,[1 1 1]),6,1) ,...
'UniformOutput',false), ...
6,[1 1 1]);

cellfun(@patch,XYZ{1},XYZ{2},XYZ{3}, repmat({color},6,1),repmat({'FaceAlpha'},6,1),repmat({clearance},6,1));

