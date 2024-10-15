clear all
close all

%% Create the course to test on
% Course 1: 0 obstacle
% Course 2: 1 obstacle
% Course 3: 3 obstacles (with corner)
% Course 4: 5 obstacles (with corner)
% Course 5: 12 obstacles (uniformly distributed)

comp_time1 = [];
comp_time2 = [];

for courseNum = 1:4  % Loop through courses 1 to 4
    testCourse = defaultCourse(courseNum);  % Get the course based on the courseNum

    %% Call the setup function
    param = mySetup(testCourse.shape);

    %% Call the initial guess from graph method
    param.iniGue = initialGuessHybAstar(param);

    %% Call path planner
    tic;
    planTraj1 = trajPlanner_condense(param);
    t1 = toc;

    tic;
    planTraj2 = trajPlanner_sparse(param);
    t2 = toc;

    comp_time1 = [comp_time1; t1];
    comp_time2 = [comp_time2; t2];
end

close all;

%% Plot the comparison of computation times
figure;
hold on;
% Create bar chart for both planners
bar(1:4, [comp_time1, comp_time2], 'grouped');
% Label the axes
xlabel('Course Number');
ylabel('Computation Time (seconds)');
% Set x-axis ticks to only show integers
xticks(1:4);
% Add legend for the two planners
legend('Single shooting', 'Mutiple shooting');
grid on;
hold off;

