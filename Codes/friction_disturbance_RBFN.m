function [sys, x0, str, ts] = friction_disturbance_RBFN(t, x, u, flag)
    switch flag
        case 0
            [sys, x0, str, ts] = mdlInitializeSizes();
        case 1
            sys = mdlDerivatives(t, x, u);
        case 3
            sys = mdlOutputs(t, x, u);
        case {2, 4, 9}
            sys = [];
        otherwise
            error(['Unhandled flag = ', num2str(flag)]);
    end
end

% Initialize sizes
function [sys, x0, str, ts] = mdlInitializeSizes()
    sizes = simsizes;
    sizes.NumContStates  = 6*6; % RBF weights (Wf)
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 6;   % Compensation torques (must be a 6x1 real vector)
    sizes.NumInputs      = 6;   % Joint velocity q_dot (6x1)
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    x0 = zeros(6*6,1); % Initialize RBF weights ???
    str = [];
    ts = [0 0]; % Continuous system
end

% Compute weight updates using gradient descent
function sys = mdlDerivatives(t, x, u)
    q_dot = u(1:6); % Joint velocities

    % Define friction matrices (diagonal)
    f_c = diag([0.1, 0.15, 0.2, 0.1, 0.15, 0.2]); % Coulomb friction
    f_v = diag([0.01, 0.02, 0.03, 0.01, 0.02, 0.03]); % Viscous friction

    % Compute actual friction force
    friction_force = f_c * sign(q_dot) + f_v * q_dot;

    % Generate disturbance terms
    d = 5 * [sin(100*t) + 1 + 5*q_dot(1);
             cos(100*t) + 3*q_dot(2);
             sin(100*t) + 1 + 5*q_dot(3);
             cos(100*t) + 3*q_dot(4);
             sin(100*t) + 1 + 5*q_dot(5);
             cos(100*t) + 3*q_dot(6)];

    % Define RBF centers and widths
    c = linspace(-5, 5, 6)'; % Column vector for centers
    sigma = 1.0; % Width

    % Compute RBF features (Gaussian basis functions)
    Phi = exp(-((q_dot - c).^2) / (2 * sigma^2)); % Should be 6x6

    % Extract current RBF weights
    W_f = reshape(x(1:36), [6, 6]); % Reshape weight vector

    % Compute RBF approximation
    friction_rbf = W_f' * Phi; % This should be 6x1

    % Error signal
    error_signal = friction_force + d - friction_rbf;

    % Gradient descent update
    learning_rate = 0.01;
    dW_f = learning_rate * (Phi * error_signal'); % Update rule

    % Ensure output is a column vector
    sys = dW_f(:);
end

% Compute output (compensation torque)
function sys = mdlOutputs(t, x, u)
    q_dot = u(1:6); % Joint velocities

    % Define RBF centers and widths
    c = linspace(-5, 5, 6)';
    sigma = 1.0;

    % Compute RBF features
    Phi = exp(-((q_dot - c).^2) / (2 * sigma^2)); % Should be 6x6

    % Extract trained RBF weights
    W_f = reshape(x(1:36), [6, 6]); % Ensure 6x6

    % Compute estimated compensation torque
    compensation_torque = W_f' * Phi; % Should be 6x1

    % Ensure output is a real-valued 6x1 column vector
    sys = reshape(compensation_torque, [6,1]);
end
