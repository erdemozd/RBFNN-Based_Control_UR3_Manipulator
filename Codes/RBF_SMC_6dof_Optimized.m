%% --- RBF-SMC Global Approximation ---
function [sys,x0,str,ts] = RBF_SMC_6dof_Optimized(t,x,u,flag)
global node c b Gamma 

switch flag
    case 0 
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled Flag = ', num2str(flag)]);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
global node c b Gamma

node = 7;
b = 3;
Gamma = 5 * eye(6);

% 6-7-1 RBF structure → input z = [e; de] = 12D
c = repmat([-1.5 -1 -0.5 0 0.5 1 1.5], 12, 1);  % size 12x7

sizes = simsizes;
sizes.NumContStates = 6 * node;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 7;  % 6 torques + 1 norm output
sizes.NumInputs = 30;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;

sys = simsizes(sizes);
x0 = zeros(6 * node, 1);
str = [];
ts = [];
end

function sys = mdlDerivatives(t, x, u)
global node c b Gamma

qd = u(1:6); dqd = u(7:12); ddqd = u(13:18);
q = u(19:24); dq = u(25:30);

e = qd - q;
de = dqd - dq;
r = de + Gamma * e;

% Optional: normalize errors for balanced RBF input
scale_e = [1 1 1 0.8 0.7 0.6];
scale_de = [1 1 1 0.8 0.7 0.6];
z = [e ./ scale_e'; de ./ scale_de'];  % 12x1

% Shared basis functions
h = zeros(node,1);
for j = 1:node
    h(j) = exp(-norm(z - c(:, j))^2 / (b * b));
end

learn_rate = 5;
decay_rate = 0.01;
sys = zeros(6 * node, 1);

for i = 1:6
    for j = 1:node
        idx = (i-1)*node + j;
        sys(idx) = learn_rate * h(j) * r(i) - decay_rate * x(idx);
    end
end
end

function sys = mdlOutputs(t, x, u)
global node c b Gamma

qd = u(1:6); dqd = u(7:12); ddqd = u(13:18);
q = u(19:24); dq = u(25:30);

e = qd - q;
de = dqd - dq;

r = de + Gamma * e;

% Joint-wise scaling
r_scale = [1, 1, 1, 0.8, 0.7, 0.5]';
r = r .* r_scale;

% Normalize inputs
scale_e = [1 1 1 0.8 0.7 0.6];
scale_de = [1 1 1 0.8 0.7 0.6];
z = [e ./ scale_e'; de ./ scale_de'];  % 12x1

% Extract weights
W_f = reshape(x(1:6*node), node, 6)';  % 6x7

% Shared basis functions
h = zeros(node,1);
for j = 1:node
    h(j) = exp(-norm(z - c(:, j))^2 / (b * b));
end

% Approximate function output
fn = zeros(6,1);
for i = 1:6
    fn(i) = W_f(i,:) * h;
end

% Control Law
Kv = diag([10, 10, 3, 2, 1, 0.5]);
epN = 0.2; bd = 0.05; epsilon = 0.1;
v = -(epN + bd) * tanh(r / epsilon);

tol = fn + Kv * r - v;

% Clamp r for stability
r = min(max(r, -5), 5);

% Clamp torque internally
maxTorque = transpose([54 54 28 9 9 9]); % see: https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/
tol = min(max(tol, -maxTorque), maxTorque);

% Optional: rate-limiting (anti-spike smoothing)
persistent prev_tol
if isempty(prev_tol)
    prev_tol = zeros(6,1);
end
alpha = 0.9;
tol = alpha * prev_tol + (1 - alpha) * tol;
prev_tol = tol;

% Output
sys = zeros(7,1);
sys(1:6) = tol;
sys(7) = norm(fn);
end
