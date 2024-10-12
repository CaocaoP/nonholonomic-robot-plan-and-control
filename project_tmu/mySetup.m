function param = mySetup(shape)

%% Modify the following function for your setup function
% Extract parameters from the defaultCourse.m
param.constraint.xy = shape.constraints.rect;
param.constraint.ellipses = shape.constraints.ellipses;
param.target = shape.target;
param.start = shape.start;
param.vRange = [-5 5];
param.omegaRange = [-pi/2 pi/2];

% Define the prediction horizon
param.Ts = 0.1;     % This is a sample way to set your sampling time
param.Tf = 5;
param.N = ceil(param.Tf/param.Ts) + 1;

param.x0 =  param.start;
param.u0 = zeros(2,1);

% Define the dimension of the system
param.n = length(param.x0);
param.p = param.n;
param.m = length(param.u0);

% Create a nonlinear MPC controller
param.nlmpcobj = nlmpc(param.n, param.p ,param.m);

% Specify the controller sample time and horizons
param.nlmpcobj.Ts = param.Ts;

% Specify the prediction model state function and the Jacobian of the state function
% param.nlmpcobj.Model.StateFcn = @(x,u) u;
param.nlmpcobj.Model.StateFcn = @(in1,in2)stateFcnBase(in1,in2);
param.nlmpcobj.Jacobian.StateFcn = @(in1,in2)stateJacobianFcnBase(in1,in2);
param.nlmpcobj.Model.IsContinuousTime = true;

% Define weights for the cost function
param.nlmpcobj.Weights.OutputVariables = [10 10 10];
param.nlmpcobj.Weights.ManipulatedVariables = [1 5];

% Add constraints on inputs
% For velocity
param.nlmpcobj.MV(1).Min = param.vRange(1);
param.nlmpcobj.MV(1).Max = param.vRange(2);
% For omega
param.nlmpcobj.MV(2).Min = param.omegaRange(1);
param.nlmpcobj.MV(2).Max = param.omegaRange(2);

param.nlmpcobj.Optimization.SolverOptions.FunctionTolerance = 0.001;
param.nlmpcobj.Optimization.SolverOptions.StepTolerance = 0.001;
param.nlmpcobj.Optimization.SolverOptions.MaxIter = 50;
param.nlmpcobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;

% Validate the prediction model and custom functions
validateFcns(param.nlmpcobj, param.x0, param.u0);
end % End of mySetup

