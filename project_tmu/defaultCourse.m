function course = defaultCourse(courseNum)
% Generate the default course
if courseNum == 0
    constraints.rect = [0.00, 0.00;
        0.00, 15.0;
        15.0, 15.0;
        15.0, 00.0];

    constraints.ellipses = {};

    start  = [pi/2, 1.00, 1.00];
    target = [0.00, 14.0, 14.0];

elseif courseNum == 1
    constraints.rect = [0.00, 0.00;
        0.00, 15.0;
        15.0, 15.0;
        15.0, 00.0];

    ellipse.a = 2.0;
    ellipse.b = 2.0;
    ellipse.xc = 4.0;
    ellipse.yc = 4.0;
    constraints.ellipses{1} = ellipse;

    start  = [pi/2, 1.00, 1.00];
    target = [0.00, 14.0, 14.0];

elseif courseNum == 2
    % Define rectangular boundary
    constraints.rect = [0.00, 0.00;
        0.00, 15.0;
        15.0, 15.0;
        15.0, 00.0];

    ellipse1.a = 2.0;
    ellipse1.b = 2.0;
    ellipse1.xc = 4.0;
    ellipse1.yc = 4.0;
    constraints.ellipses{1} = ellipse1;

    ellipse2.a = 3.0;
    ellipse2.b = 1.5;
    ellipse2.xc = 10.0;
    ellipse2.yc = 6.0;
    constraints.ellipses{2} = ellipse2;

    ellipse3.a = 1.5;
    ellipse3.b = 3.0;
    ellipse3.xc = 10.0;
    ellipse3.yc = 12.0;
    constraints.ellipses{3} = ellipse3;

    start  = [pi/2, 1.00, 1.00];
    target = [0.00, 14.0, 14.0];

elseif courseNum == 3
    constraints.rect = [0.00, 0.00;
        0.00, 15.0;
        15.0, 15.0;
        15.0, 00.0];

    % First ellipse
    ellipse1.a = 2.0;
    ellipse1.b = 2.0;
    ellipse1.xc = 4.0;
    ellipse1.yc = 4.0;
    constraints.ellipses{1} = ellipse1;

    % Second ellipse
    ellipse2.a = 4.0; % Reduced semi-major axis
    ellipse2.b = 1.0; % Reduced semi-minor axis
    ellipse2.xc = 9.0; % Moved closer to the first ellipse
    ellipse2.yc = 4.0;
    constraints.ellipses{2} = ellipse2;

    % Third ellipse
    ellipse3.a = 1.0; % Reduced semi-major axis
    ellipse3.b = 4.0; % Reduced semi-minor axis
    ellipse3.xc = 7.5; % Moved closer to others
    ellipse3.yc = 13.0;
    constraints.ellipses{3} = ellipse3;

    % Fourth ellipse
    ellipse4.a = 2.0; % Reduced semi-major axis
    ellipse4.b = 0.8; % Reduced semi-minor axis
    ellipse4.xc = 6.0; % Moved closer
    ellipse4.yc = 9.0;
    constraints.ellipses{4} = ellipse4;

    % Fifth ellipse
    ellipse5.a = 1.0; % Reduced semi-major axis
    ellipse5.b = 4.0; % Reduced semi-minor axis
    ellipse5.xc = 11.0; % Moved closer to fourth ellipse
    ellipse5.yc = 8.0;
    constraints.ellipses{5} = ellipse5;

    % Define start and target positions
    start  = [pi/2, 1.00, 1.00];
    target = [0.00, 14.0, 14.0];

elseif courseNum == 4
    % Define rectangular boundary
    constraints.rect = [0.00, 0.00;
        0.00, 15.0;
        15.0, 15.0;
        15.0, 00.0];

    % Number of ellipses
    numEllipses = 12;

    % Define grid dimensions for uniform distribution
    numRows = 3; % Number of rows of ellipses
    numCols = 4; % Number of columns of ellipses

    % Define a margin
    margin = 1;

    % Ensure grid covers the rectangle area
    xGridSpacing = (max(constraints.rect(:,1)) - min(constraints.rect(:,1)) - 2*margin) / numCols;
    yGridSpacing = (max(constraints.rect(:,2)) - min(constraints.rect(:,2)) - 2*margin) / numRows;

    % Generate ellipses uniformly in the grid
    ellipseIndex = 1;
    for row = 1:numRows
        for col = 1:numCols
            if ellipseIndex > numEllipses
                break;
            end

            % Calculate center position for the ellipse
            xc = min(constraints.rect(:,1)) + margin + (col - 0.5) * xGridSpacing;
            yc = min(constraints.rect(:,2)) + margin + (row - 0.5) * yGridSpacing;

            % Assign semi-major and semi-minor axes (randomly within limits)
            a = 1.0;
            b = 1.0;
            % Create the ellipse and assign to the constraint
            ellipse.a = a;
            ellipse.b = b;
            ellipse.xc = xc;
            ellipse.yc = yc;
            constraints.ellipses{ellipseIndex} = ellipse;

            % Increment the index
            ellipseIndex = ellipseIndex + 1;
        end
    end

    % Define start and target positions
    start  = [pi/2, 5.00, 1.00];
    target = [0.00, 14.0, 14.0];
end

shape.constraints = constraints;

shape.start       = start;
shape.target      = target;

course.shape = shape;
course.num = courseNum;
end