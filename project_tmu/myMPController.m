function u = myMPController(param, y)
%% Design your controller here
% Define exceeding horizon and refence 
exHorizon = 10;

if ~isempty(param.planTraj)
Xref = param.planTraj.state;  % Generated reference signal
Uref = param.planTraj.input;    % Initial guess for MPC
Xref = [Xref'; repmat(param.target, exHorizon, 1)];
Uref = [Uref'; repmat(param.u0', exHorizon, 1)];
else
Xref = [repmat(param.target, param.N + exHorizon, 1)];
X0 = param.iniGue;
end

% Define persistent
persistent lastMV N options;
if isempty(lastMV)
    lastMV = param.u0;
    N = param.N + exHorizon;
    options = nlmpcmoveopt;
    options.MVTarget = param.u0';
    if ~isempty(param.planTraj)
    options.X0 = Xref;
    options.MV0 = Uref;
    else
    options.X0 = X0;
    end
end

% Define predicted horizon
param.nlmpcobj.PredictionHorizon = N;
param.nlmpcobj.ControlHorizon = N;

% Add constraints on states
param.nlmpcobj.Optimization.CustomIneqConFcn = @(X,U,e,data)myIneqConFunction(X,U,e,data,param.constraint);

% MPC Implementation
param.nlmpcobj.Optimization.SolverOptions.Algorithm = 'sqp';
param.nlmpcobj.Optimization.RunAsLinearMPC = 'timevarying';
ref = Xref(end-N+1:end, :);
myState = y;
[U, options, ~] = nlmpcmove(param.nlmpcobj,myState,lastMV,ref,[],options);
lastMV = U;

% figure
% Plot the course, start, target, and trajectory
% plotCourse(param);
% hold on;
% plot(param.start(2), param.start(3), 'mo');
% plot(param.target(2), param.target(3), 'm*');
% plot(options.X(:, 2), options.X(:, 3), 'r*');
% hold off;

% Extract control
u = U(1:param.m);
N = N - 1;
end % End of myMPController


%% Local functions
function cineq = myIneqConFunction(X,U,e,data,constraint)
cineq = [];
% Define inequality constraints for elliptical obstacles
posX = X(:,2);
posY = X(:,3);
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