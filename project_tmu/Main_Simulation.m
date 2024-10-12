%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This file will run a complete simulation of the nonholonomic robot

% Authors: Pinyu Cao
% Revision: 2024.10
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%% Define all the selectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Selector of graph method for the initial guess (path)
% 'prm' - Probabilistic Roadmaps 
% 'astar' - HybridAStar
igSelect = 'astar';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Selector of the optimisation based path planner
% 'sparse' - Sparse nonlinear optimisation
% 'condense' - Condense nonlinear optimisation
% 'off' - No trajectory planning (MPC only)
tpSelect = 'sparse';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Selector of the controller
% 'mpc' - Model predictive controller
% 'nlc' - Nonlinear feedforward + feedback controller
ctrlSelect = 'nlc';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Call the setup function
param = mySetup(testCourse.shape);

%% Call the initial guess from graph method
switch igSelect
    case 'astar'
        param.iniGue = initialGuessHybAstar(param);
    case 'prm'
        param.iniGue = initialGuessPRM(param);
    otherwise
        disp('Invalid initial guess selector. Please choose ''astar'', or ''prm''.');
end

%% Call the trajectory planner
switch tpSelect
    case 'sparse'
        param.planTraj = trajPlanner_sparse(param);
    case 'condense'
        param.planTraj = trajPlanner_condense(param);
    case 'off'
        param.planTraj = [];
    otherwise
        disp('Invalid trajectory planner selector. Please choose ''sparse'', ''condense'', or ''off''.');
end

%% Declare other simulation parameters & Setup the simulation
T = 5;           % Set the simulation final time
tstep = 0.001;   % Step size for simulation.

Ts = param.Ts;   % Sampling time for MPC

hw = waitbar( 0,'Please wait...' ); % create waiting bar
warning( 'on' );

odeOpts = odeset( 'RelTol', 1e-3, 'MaxStep', 0.001 ); % ODE Options

x = [testCourse.shape.start(1,1), testCourse.shape.start(1,2), testCourse.shape.start(1,3)]; % Initial conditions
y = x';

% Setup variables to hold the results
timeConcat    = 0;
inputsConcat  = [];
statesConcat  = x;

%% Iterate for the simulation time
for t=0:Ts:T 
    waitbar( t/T, hw, 'Please wait...' );
    
    % Call the MPC controller function
    switch ctrlSelect
        case 'mpc'
            u = myMPController(param,y);
        case 'nlc'
            u = myNLController(param,y);
    end
        
    % Setup the simulation model & Simulate
    usat = zeros(2,1);
    usat(1) = min(max(u(1), param.vRange(1)), param.vRange(2));
    usat(2) = min(max(u(2), param.omegaRange(1)), param.omegaRange(2));

    mod = @(t, state) stateFcnBase(state, usat);

    [tt, x] = ode45( mod, t:tstep:t + Ts, x(end,:), odeOpts );
    
    % The output is simply all the states
    y = x(end,:)';

    % Keep variables for analysis
    % We ignore the first row to avoid data overlap
    timeConcat    = [timeConcat;    tt(2:end, end)];
    statesConcat  = [statesConcat;  x(2:end, :)];
    inputsConcat  = [inputsConcat;  u'.*ones(size(tt, 1)-1, 1)];
    
end
close(hw);

plotCourse(param);
hold on;
plot(statesConcat(:,2), statesConcat(:,3), 'c-', LineWidth = 1.5);