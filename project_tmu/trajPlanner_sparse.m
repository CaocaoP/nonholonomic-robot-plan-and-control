%% Main function
function planTraj = trajPlanner_sparse(param)
% Use a sparse form of optimisation (multiple shooting),
% w = [u_0, u_1, ..., u_N-1, x_0, x_1, ...,x_N]

% Initial guess for optimization
X0 = param.iniGue';
U0 = generateInputFromState(X0, param.n, param.m, param.N, param.Ts, param.vRange, param.omegaRange);
w0 = [reshape(U0, param.m * param.N, 1); reshape(X0,param.n*(param.N+1),1)];
% w0 = zeros((param.m+param.n)*param.N+param.n,1);

% Define the cost function
fun = @(w) mycostfun(w, param.n, param.m, param.N, param.target');

% Define the linear equality condition
[Aeq, beq] = leqcon(param.n, param.m, param.N, param.start');

% Define the linear inequality condition
[A, b] = lineqcon(param.n, param.m, param.N, param.target', param.constraint, ...
    param.vRange, param.omegaRange);

% Define the nonlinear condition
nonlcon = @(w) mynlcon(w, param.n, param.m, param.N, param.Ts, param.constraint);

% Solve the optimization problem with fmincon
options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
                       'TolCon', 1e-4, ... % Relax constraint tolerance
                       'StepTolerance', 1e-8, ... % Tighten step tolerance if needed
                       'MaxIterations', 5000, ...
                       'MaxFunctionEvaluations', 10000, ...
                       'Display', 'iter-detailed'); % Show detailed iteration output

[W, fval, exitflag, ~] = fmincon(fun, w0, A, b, Aeq, beq, [], [], nonlcon, options);
assignin('base', 'exitflag', exitflag);

% Extract the trajectory X(:,0:N)
planTraj.input = reshape(W(1:param.m*param.N),param.m,param.N);
planTraj.state = reshape(W(param.m*param.N+1:end),param.n,param.N+1);

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

function f = mycostfun(w, n, m, N, target)
% Define weighting matrices
Q = diag([10,10,10]); % State weighting matrix
R = diag([1, 5]); % Input weighting matrix
QQ = kron(eye(N+1), Q); % Stack to QQ
RR = kron(eye(N), R); % Stack to RR
H = blkdiag(RR, QQ); % Add terminal state cost

% Reference trajectory
w_ref = [zeros(m*N,1); repmat(target, N+1, 1)];

% Cost function with regularization
f = 0.5 * (w - w_ref)' * H * (w - w_ref);
end

function [Aeq, beq] = leqcon(n, m, N, start)
% Initial value constraint (initial state is fixed to 'start')
Aeq_inival = zeros(n, (m+n)*N + n);
k = m*N +1;
Aeq_inival(1:n, k:k+n-1) = eye(n);
beq_inival = start;

% Combine all equality constraints into Aeq and beq
Aeq = Aeq_inival;
beq = beq_inival;
end

function [A, b] = lineqcon(n, m, N, target, constraint, vRange, omegaRange)
% Input contraint
lb = [vRange(1); omegaRange(1)];
ub = [vRange(2); omegaRange(2)];
A_input = zeros(2*m*N, (m+n)*N + n);
b_input = zeros(2*m*N, 1);
A_input(1:m*N,1:m*N) = eye(m*N);
A_input(m*N+1:end,1:m*N) = -eye(m*N);
b_input(1:m*N,1) = repmat(ub,N,1);
b_input(m*N+1:end,1) = -repmat(lb,N,1);

% Terminal constraint (only at step N)
eps = 1e-4;
A_term = zeros(2*n, (m+n)*N + n);
k = 1+N*(m+n);
A_term(1:2*n, k:k+n-1) = [eye(n); -eye(n)];
b_term = [target+eps; -target+eps];

% Box constraint
A_box = zeros(4*(N+1), (m+n)*N+n);
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
Dt = [0, alpha1, alpha2;
      0,-alpha1,-alpha2;
      0,  beta1,  beta2;
      0, -beta1, -beta2]; % Matrix form in one interval
bt = [alphaS+alphaP;
      alphaS-alphaP;
      betaS+betaP;
      betaS-betaP]; % Matrix form in one interval
A_box(:,m*N+1:end) = kron(eye(N+1),Dt);
b_box = repmat(bt,N+1,1);

% Combine all equality constraints into Aeq and beq
A = [A_input; A_term; A_box];
b = [b_input; b_term; b_box];
end

function [cineq, ceq] = mynlcon(w, n, m, N, Ts, constraint)
cineq = []; % Initialize inequality constraints
ceq = []; % Initialize equality constraints

% Loop over each step of the trajectory
for i = 1:N
    % Extract indices for current state, input, and next state
    ku = 1 + (i-1)*m; % Calculate the index for u
    kx = 1 + N*m + (i-1)*n; % Calculate the index for x
    
    x = w(kx : kx+n-1);
    u = w(ku : ku+m-1);
    x_next = w(kx+n : kx+2*n-1);

    % Midpoint rule update to estimate the next state
    ctans = stateTransition(x, u, Ts) - x_next;

    % Accumulate the equality constraints
    ceq = [ceq; ctans];

    % Extract position from the next state for obstacle constraints
    posX = x_next(2); % X position component
    posY = x_next(3); % Y position component

    % Define inequality constraints for elliptical obstacles
    for j = 1:length(constraint.ellipses)
        a = constraint.ellipses{j}.a;   % Semi-major axis of ellipse
        b = constraint.ellipses{j}.b;   % Semi-minor axis of ellipse
        xc = constraint.ellipses{j}.xc; % Center X coordinate of ellipse
        yc = constraint.ellipses{j}.yc; % Center Y coordinate of ellipse

        % Inequality constraint for ellipses: point must be outside
        celps = 1.1 - ((posX - xc)^2) / (a^2) - ((posY - yc)^2) / (b^2);
        cineq = [cineq; celps];
    end
end
end



