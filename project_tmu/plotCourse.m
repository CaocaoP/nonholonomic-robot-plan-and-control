function [ fig ] = plotCourse( param )
%PLOTCOURSE Summary of this function goes here
%   Detailed explanation goes here

%% Extract the shape in the course
constraint = param.constraint;

%% Create the figure
figure;
hold on;
%% Add the constraint boundaries
v = [constraint.xy];
f = [1 2 3 4];
patch('Faces',f,'Vertices',v, ...
    'EdgeColor','blue','FaceColor','none','LineWidth',2);

% Save the limits for the rectangle so we can restore them after adding the ellipses
xl = xlim();
yl = ylim();

% Add the ellipses if there are any
if( isfield( constraint, 'ellipses' ) && ~isempty( constraint.ellipses ) )
    for i=1:1:length( constraint.ellipses ) 
        ellipse = constraint.ellipses{i};

        t = -pi:0.01:pi;
        x = ellipse.xc + ellipse.a * cos( t );
        y = ellipse.yc + ellipse.b * sin( t );

        pe = patch(x, y, 'r');
        pe.FaceAlpha = 0.3;
    end
end
    
% Restore the previous plot limits
xlim( xl );
ylim( yl );

%% Add the starting point
plot( param.start(2), param.start(3), 'mo' );
quiver(param.start(2), param.start(3), ...
    0.9*cos(param.start(1)), 0.9*sin(param.start(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);


%% Add the target point (with epsilon size)
plot( param.target(2), param.target(3), 'm*' );
quiver(param.target(2), param.target(3), ...
    0.9*cos(param.target(1)), 0.9*sin(param.target(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);


%% Set the right limits to the visualization
yl = ylim();
xl = xlim();

ydiff = diff( yl );
xdiff = diff( xl );

if( ydiff == xdiff )
    % The axis are already equal
    newyl = yl;
    newxl = xl;
elseif( ydiff > xdiff )
    % The y axis is larger than x - expand x
    expand = ( ydiff - xdiff ) / 2;
    
    newxl = xl + [-expand, expand];
    newyl = yl;
else
    % The x axis is larger than y - expand y
    expand = ( xdiff - ydiff ) / 2;
    
    newxl = xl;
    newyl = yl + [-expand, expand];
end

% Add a slight border around the graph
ylim( newyl + [-0.01, 0.01] );
xlim( newxl + [-0.01, 0.01] );

axis square;

end
