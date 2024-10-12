%% Main functions
function planTraj = trajPlanner_condense(param)
% Use a condense form of optimisation (single shooting), 
% w = [u_0, u_1, u_2, ..., u_N-1]

% Initial guess for optimization
X0 = param.iniGue';
U0 = generateInputFromState(X0, param.n, param.m, param.N, param.Ts, param.vRange, param.omegaRange);
w0 = reshape(U0, param.m * param.N, 1);
% w0 = zeros(param.m*param.N,1);

% Define state trajectory function
X = @(w) solveInitialValProb(param.start', param.n, param.m, param.N, param.Ts, w);

% Define the cost function
fun = @(w) mycostfun(w, X, param.n, param.N, param.target');

% Define the nonlinear condition
nonlCon = @(w) mynlcon(w, X, param.n, param.m, param.N, param.target',param.constraint);

% Define the lower and upperbound
lB = repmat([param.vRange(1); param.omegaRange(1)], param.N, 1);
uB = repmat([param.vRange(2); param.omegaRange(2)], param.N, 1);

% Nonlinear optimisation for trajectory planning
options = optimoptions('fmincon', 'Algorithm', 'active-set', ...
                       'TolCon', 1e-4, ... % Relax constraint tolerance
                       'StepTolerance', 1e-8, ... % Tighten step tolerance if needed
                       'MaxIterations', 5000, ...
                       'MaxFunctionEvaluations', 10000, ...
                       'Display', 'iter-detailed'); % Show detailed iteration output

[w, ~, exitflag, ~] = fmincon(fun, w0, [], [], [], [], lB, uB, nonlCon, options);
disp(exitflag);
 
% Extract state trajectory from optimized controls
X_val = solveInitialValProb(param.start, param.n, param.m, param.N, param.Ts, w);

planTraj.input = reshape(w, param.m, param.N); 
planTraj.state = reshape(X_val, param.n, param.N + 1);

% Plot the course, start, target, and trajectory
plotCourse(param);
hold on;
plot(param.start(2), param.start(3), 'mo');
plot(param.target(2), param.target(3), 'm*');
plot(planTraj.state(2, :), planTraj.state(3, :), 'r*');
hold off;
end

%% Local functions
function x_next = stateTransition(x, u, Ts)
    % Simulate the next state using the Midpoint method
    k1 = stateFcnBase(x, u);
    x_mid = x + (Ts / 2) * k1;
    k2 = stateFcnBase(x_mid, u);
    x_next = x + Ts * k2;
end

function U = generateInputFromState(X, n, m, N, Ts, vRange, omegaRange)
    % Initialize control input sequence
    U = zeros(m, N);
     
    % Generate control inputs based on state trajectory
    u0 = zeros(m, 1);
    for i = 1:N
        x_curr = X(:, i);    % Current state
        x_next = X(:, i + 1);    % Next state

        % Use an optimization to find control input
        objFun = @(u) norm(stateTransition(x_curr, u, Ts) - x_next);
        lb = [vRange(1); omegaRange(1)];
        ub = [vRange(2); omegaRange(2)];

        u_opt = fmincon(objFun, u0, [], [], [], [], lb, ub);

        % Store control input and warm starting
        u0 = u_opt;
        U(:, i) = u_opt;
    end
end

function X = solveInitialValProb(x0, n, m, N, Ts, U)
% Initialize state sequence
X = zeros(n, N + 1);
X(:, 1) = x0;

% Solve the initial value problem
for i = 1:N
    x = X(:, i);
    u = U(m * (i - 1) + 1 : m * i, 1);
    x_next = stateTransition(x, u, Ts);
    X(:,i+1) = x_next;
end
end

function f = mycostfun(w, Xfun, n, N, target)
% Define state and input trajectories
X = reshape(Xfun(w), (N+1)*n, 1);
U = w;

% Define weighting matrices
Q = diag([10, 10, 10]);
R = diag([1, 5]);

% Stack weighting matrices
H_Q = kron(eye(N + 1), Q);
H_R = kron(eye(N), R);

% Define reference trajectory
X_ref = repmat(target, N + 1, 1);

% Calculate cost function
f = 0.5 * ((X - X_ref)' * H_Q * (X - X_ref) + U' * H_R * U);
end

function [cineq, ceq] = mynlcon(w, Xfun, n, m, N, target, constraint)
cineq = [];
ceq = [];

% Define state and input trajectories
X = Xfun(w);
U = reshape(w, m, N);

% Terminal constraint (at N)
x_term = X(:, end);
ceq = [ceq; x_term - target];

% Define inequality constraints for elliptical obstacles
posX = X(2,:);
posY = X(3,:);
for i = 1:length(constraint.ellipses)
    a = constraint.ellipses{i}.a;   % Semi-major axis of ellipse
    b = constraint.ellipses{i}.b;   % Semi-minor axis of ellipse
    xc = constraint.ellipses{i}.xc; % Center X coordinate of ellipse
    yc = constraint.ellipses{i}.yc; % Center Y coordinate of ellipse

    % Inequality constraint for ellipses: point must be outside
    celps = 1.1 - ((posX - xc).^2) / (a^2) - ((posY - yc).^2) / (b^2);
    cineq = [cineq; celps];
end

% Define the box constraint
c = constraint.xy;
a = (c(1,1)+c(3,1))/2;
b = (c(1,2)+c(3,2))/2;
alpha1 = c(2,1) - c(1,1);
alpha2 = c(2,2) - c(1,2);
alphaS = 0.5*(alpha1^2 + alpha2^2);
alphaP = a*alpha1 + b*alpha2;
beta1 = c(4,1) - c(1,1);
beta2 = c(4,2) - c(1,2);
betaS = 0.5*(beta1^2 + beta2^2);
betaP = a*beta1 + b*beta2;

box1 = alpha1*posX+alpha2*posY-alphaS-alphaP;
box2 = -alpha1*posX-alpha2*posY-alphaS+alphaP; 
box3 = beta1*posX+beta2*posY-betaS-betaP;
box4 = -beta1*posX-beta2*posY-betaS+betaP;
cineq = [cineq; box1; box2; box3; box4];
end