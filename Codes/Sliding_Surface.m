function [sys, x0, str, ts] = Sliding_Surface(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ', num2str(flag)]);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 6;   % 6 joints
sizes.NumInputs = 30;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;

sys = simsizes(sizes);
x0 = [];
str = [];
ts = [];
end

function sys = mdlOutputs(t,x,u)
qd = u(1:6); dqd = u(7:12); ddqd = u(13:18);  % Desired trajectories
q = u(19:24); dq = u(25:30);  % Actual joint trajectories

% Error computation
e = qd - q;
de = dqd - dq;

% Scaling factors for normalization
scale_e = [1 1 1 0.8 0.7 0.6]';     % You can tune these
scale_de = [1 1 1 0.8 0.7 0.6]';

% Normalize the errors
e_norm = e ./ scale_e;
de_norm = de ./ scale_de;

% Sliding Surface
lambda = 3 * eye(6);
r = de_norm + lambda * e_norm;

sys = r;  % Return 6x1 vector
end


