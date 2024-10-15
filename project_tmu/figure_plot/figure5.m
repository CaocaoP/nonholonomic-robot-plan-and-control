clear all
close all

%% Create the course to test on
% Course 0: 0 obstacle
% Course 1: 1 obstacle
% Course 2: 3 obstacles (with corner)
% Course 3: 5 obstacles (with corner)
% Course 4: 12 obstacles (uniformly distributed)
courseNum = 1;

% Run the simulation for different control strategies

[ref, time] = reference(courseNum, 'astar', 'sparse');
[states1, time1] = simulation(courseNum, 'astar', 'sparse', 'nlc');
[states2, time2] = simulation(courseNum, 'astar', 'sparse', 'mpc');
clear myMPController;
[states3, time3] = simulation(courseNum, 'astar', 'off', 'mpc');

% Close all figures
close all;

%% Plot results in a loop to avoid code repetition
state_labels = {'\phi response', 'x response', 'y response'};
legend_entries = {'Reference', 'Nonlinear', 'MPC Tracking', 'MPC Regulation'};
states = {states1, states2, states3};
times = {time1, time2, time3};

for i = 1:3
    figure('Position', [100, 100, 1200, 300]);
    hold on;
    plot(time, ref(i,:), 'c', 'LineWidth', 1.5);
    plot(times{1}, states{1}(:,i), 'r', 'LineWidth', 1.5);
    plot(times{2}, states{2}(:,i), 'g', 'LineWidth', 1.5);
    plot(times{3}, states{3}(:,i), 'b', 'LineWidth', 1.5);
    legend(legend_entries, 'Location', 'southeast', 'FontSize', 14);
    title(state_labels{i}, 'FontSize', 16);
    xlabel('Time (s)', 'FontSize', 14);
    ylabel('State', 'FontSize', 14);
    hold off;
    grid on;
    xlim([0 5]);
    set(gca, 'FontSize', 14);
end

function [ref, time] = reference(courseNum, igSelect, tpSelect)
testCourse = defaultCourse(courseNum);

%% Setup parameters based on course shape
param = mySetup(testCourse.shape);

%% Initial guess selection
param.iniGue = selectInitialGuess(igSelect, param);
    
%% Trajectory planner selection
param.planTraj = selectTrajectoryPlanner(tpSelect, param);

ref = param.planTraj.state;
time = 0:param.Ts:param.Tf+param.Ts;
end

%% Simulation Function
function [statesConcat, timeConcat] = simulation(courseNum, igSelect, tpSelect, ctrlSelect)
    testCourse = defaultCourse(courseNum);
    
    %% Setup parameters based on course shape
    param = mySetup(testCourse.shape);
    
    %% Initial guess selection
    param.iniGue = selectInitialGuess(igSelect, param);
    
    %% Trajectory planner selection
    param.planTraj = selectTrajectoryPlanner(tpSelect, param);
    
    %% Simulation parameters
    T = 5;           % Total simulation time
    tstep = 0.001;   % Simulation step size
    Ts = param.Ts;   % Sampling time
    
    odeOpts = odeset('RelTol', 1e-3, 'MaxStep', 0.001); % ODE options
    x = [testCourse.shape.start(1, 1), testCourse.shape.start(1, 2), testCourse.shape.start(1, 3)]; % Initial state
    y = x';
    
    % Initialize storage for simulation results
    timeConcat = 0;
    statesConcat = x;
    
    %% Simulation loop
    for t = 0:Ts:T
        % Select controller based on the input
        u = selectController(ctrlSelect, param, y);
        
        % Apply control inputs and simulate the system
        usat = saturateInputs(u, param);
        mod = @(t, state) stateFcnBase(state, usat);
        
        [tt, x] = ode45(mod, t:tstep:t + Ts, x(end, :), odeOpts);
        
        % Update the current state and time
        y = x(end, :)';
        timeConcat = [timeConcat; tt(2:end)];
        statesConcat = [statesConcat; x(2:end, :)];
    end
    close all;
end

%% Helper Functions
% Select the initial guess
function iniGue = selectInitialGuess(igSelect, param)
    switch igSelect
        case 'astar'
            iniGue = initialGuessHybAstar(param);
        case 'prm'
            iniGue = initialGuessPRM(param);
        otherwise
            error('Invalid initial guess selector. Choose ''astar'' or ''prm''.');
    end
end

% Select the trajectory planner
function planTraj = selectTrajectoryPlanner(tpSelect, param)
    switch tpSelect
        case 'sparse'
            planTraj = trajPlanner_sparse(param);
        case 'condense'
            planTraj = trajPlanner_condense(param);
        case 'off'
            planTraj = [];
        otherwise
            error('Invalid trajectory planner selector. Choose ''sparse'', ''condense'', or ''off''.');
    end
end

% Select the controller
function u = selectController(ctrlSelect, param, y)
    switch ctrlSelect
        case 'mpc'
            u = myMPController(param, y);
        case 'nlc'
            u = myNLController(param, y);
        otherwise
            error('Invalid controller selector. Choose ''mpc'' or ''nlc''.');
    end
end

% Saturate the control inputs
function usat = saturateInputs(u, param)
    usat = zeros(2, 1);
    usat(1) = min(max(u(1), param.vRange(1)), param.vRange(2));
    usat(2) = min(max(u(2), param.omegaRange(1)), param.omegaRange(2));
end
