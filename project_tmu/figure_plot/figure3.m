clear all
close all

%% Create the course to test on
% Course 1: 0 obstacle
% Course 2: 1 obstacle
% Course 3: 3 obstacles (with corner)
% Course 4: 5 obstacles (with corner)
% Course 5: 12 obstacles (uniformly distributed)

courseNum = 4;
testCourse = defaultCourse(courseNum);

%% Call the setup function
param = mySetup(testCourse.shape);

%% Call the initial guess from graph method
param.iniGue = initialGuessHybAstar(param);

%% Call path planner
planTraj1 = trajPlanner_condense(param);
planTraj2 = trajPlanner_sparse(param);

close all;

hold on;
plotCourse(param);
h1 = plot(planTraj1.state(2, :), planTraj1.state(3, :), 'r*');
h2 = plot(planTraj2.state(2, :), planTraj2.state(3, :), 'c*');
quiver(param.start(2), param.start(3), ...
    0.9*cos(param.start(1)), 0.9*sin(param.start(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);
quiver(param.target(2), param.target(3), ...
    0.9*cos(param.target(1)), 0.9*sin(param.target(1)), ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 5);

xlabel('X');
ylabel('Y');
legend([h1, h2], {'Single shooting', 'Mutiple shooting'});
hold off;