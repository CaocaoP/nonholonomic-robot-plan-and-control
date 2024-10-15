clear all
close all

%% Create the course to test on
% Course 1: 0 obstacle

courseNum = 1;
testCourse = defaultCourse(courseNum);
shape = testCourse.shape;

param.constraint.xy = shape.constraints.rect;
param.constraint.ellipses = shape.constraints.ellipses;
param.target = shape.target;
param.start = shape.start;
param.vRange = [-5 5];
param.omegaRange = [-pi/2 pi/2];
param.Ts = 0.1;
param.Tf = 5;
param.N = ceil(param.Tf/param.Ts) + 1;

P1 = initialGuessPRM(param);
P2 = initialGuessHybAstar(param);

close all;

hold on;
plotCourse(param);
h1 = plot(P1(:,2), P1(:,3), 'r-.', 'LineWidth', 2);
h2 = plot(P2(:,2), P2(:,3), 'c-.', 'LineWidth', 2);
quiver(param.start(2), param.start(3), ...
    0.9*cos(param.start(1)), 0.9*sin(param.start(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);
quiver(param.target(2), param.target(3), ...
    0.9*cos(param.target(1)), 0.9*sin(param.target(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);

xlabel('X');
ylabel('Y');
title('Path Comparison: PRM vs Hybrid A*');
legend([h1, h2], {'PRM Path', 'Hybrid A* Path'});
hold off;