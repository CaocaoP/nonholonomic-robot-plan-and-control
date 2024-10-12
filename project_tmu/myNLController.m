function u = myNLController(param, y)
% myNLController: Nonlinear controller function
% Inputs:
%   param - A structure containing the planned trajectory (planTraj)
%   y - The current state vector [phi, x, y]
% Outputs:
%   u - Controller output [v, omega]

% Check if the planned trajectory is provided
if isempty(param.planTraj)
    disp('Trajectory planner selector must be ''sparse'', or ''condense'' for nonlinear controller.');
end

% Persistent variable N, used to track the current point on the trajectory
persistent N;
if isempty(N)
    N = 1; % Initialize N to 1
end

% Desired input from the planned trajectory: linear velocity (v_d) and angular velocity (w_d)
v_d = param.planTraj.input(1, N);
w_d = param.planTraj.input(2, N);

% Current state: angle (phi), position (x, y)
phi = y(1);
x = y(2);
y = y(3);

% Desired state: desired angle (phi_d), desired position (x_d, y_d)
phi_d = param.planTraj.state(1, N);
x_d   = param.planTraj.state(2, N);
y_d   = param.planTraj.state(3, N);

% Calculate the error vector: [phi - phi_d; x - x_d; y - y_d]
vSub = [phi - phi_d; x - x_d; y - y_d];

% Define the rotation matrix
R = [1,          0,           0;
     0,  cos(phi_d), sin(phi_d);
     0, -sin(phi_d), cos(phi_d)];

% Compute the transformed state error: [phi_e, x_e, y_e]
state_error = R * vSub;
phi_e = state_error(1);
x_e = state_error(2);
y_e = state_error(3);

% Controller gains ( > 0 )
k1 = 1;
k2 = 1;
k3 = 1;

% Compute the controller output: linear velocity (v) and angular velocity (omega)
v = (v_d - k1 * abs(v_d) * (x_e + y_e * tan(phi_e))) / cos(phi_e);
omega = w_d - (k2 * v_d * y_e + k3 * abs(v_d) * tan(phi_e)) * cos(phi_e)^2;

% Update the trajectory point index N
N = N + 1;

% Return the controller output [v, omega]
u = [v; omega];
end