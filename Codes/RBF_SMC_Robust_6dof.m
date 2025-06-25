%% --- RBF-SMC Local Approach ---
function [sys,x0,str,ts] = RBF_SMC_Robust_6dof(t,x,u,flag)
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
            error(['Unhandled flag = ',num2str(flag)]);
    end
end

%% --- Initialization Function ---
function [sys,x0,str,ts] = mdlInitializeSizes
    global node c_M c_C c_G b scale_e scale_de Fai

    node = 7;  % Number of RBF nodes

    % RBF centers for normalized input space [-1.5,1.5]
    c_M = linspace(-1.5, 1.5, node);
    c_C = linspace(-1.5, 1.5, node);
    c_G = linspace(-1.5, 1.5, node);

    b = 1.0;  % Spread factor (adjusted for normalized inputs)

    % Normalization scales for errors and sliding surface
    scale_e = [1 1 1 0.8 0.7 0.6];
    scale_de = [1 1 1 0.8 0.7 0.6];
    Fai = 3 * eye(6);

    sizes = simsizes;
    sizes.NumContStates = 18 * node;     % 6 DOF * 3 networks * nodes
    sizes.NumDiscStates = 0;
    sizes.NumOutputs = 6;                % Torque output size
    sizes.NumInputs = 36;                % qd, dqd, ddqd, q, dq, I (6 each)
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);
    x0 = zeros(18 * node, 1);
    str = [];
    ts = [0 0];
end

%% --- Weight Update (Dynamics) ---
function sys = mdlDerivatives(~,x,u)
    global node c_M c_C c_G b scale_e scale_de Fai

    qd = u(1:6); dqd = u(7:12); ddqd = u(13:18);
    q = u(19:24); dq = u(25:30);

    e = qd - q;
    de = dqd - dq;

    % Normalize errors
    en = e ./ scale_e';
    den = de ./ scale_de';

    % Sliding surface and normalize
    r = de + Fai * e;
    scale_r = [1 1 1 0.8 0.7 0.6]';
    r_norm = r ./ scale_r;

    dqr = dqd + Fai * e;
    ddqr = ddqd + Fai * de;

    % Compute RBF basis for M, C, G networks (2D input per joint: [e(i); de(i)])
    h_M = zeros(node,6);
    h_C = zeros(node,6);
    h_G = zeros(node,6);

    for i=1:6
        for j=1:node
            z = [en(i); den(i)];
            c_vec = [c_M(j); c_M(j)];  % same center for simplicity
            h_M(j,i) = exp(-norm(z - c_vec)^2 / b^2);
            h_C(j,i) = exp(-norm(z - c_vec)^2 / b^2);
            h_G(j,i) = exp(-norm(z - c_vec)^2 / b^2);
        end
    end

    % Extract weights
    W_M = reshape(x(1:6*node), node, 6)';
    W_C = reshape(x(6*node+1:12*node), node, 6)';
    W_G = reshape(x(12*node+1:end), node, 6)';

    % Learning rates and decay
    learn_rate = 0.1;    % smaller learning rate for stability
    decay_rate = 0.01;   % weight decay to avoid runaway

    T_M = learn_rate * eye(node);
    T_C = learn_rate * eye(node);
    T_G = learn_rate * eye(node);

    sys = zeros(18 * node, 1);

    for i=1:6
        for j=1:node
            idx_M = (i-1)*node + j;
            idx_C = 6*node + (i-1)*node + j;
            idx_G = 12*node + (i-1)*node + j;

            sys(idx_M) = T_M(j,j) * h_M(j,i) * ddqr(i) * r_norm(i) - decay_rate * x(idx_M);
            sys(idx_C) = T_C(j,j) * h_C(j,i) * dqr(i) * r_norm(i) - decay_rate * x(idx_C);
            sys(idx_G) = T_G(j,j) * h_G(j,i) * r_norm(i) - decay_rate * x(idx_G);
        end
    end

    % Clip weights derivatives to prevent instability
    max_w = 100; min_w = -100;
    sys = max(min(sys, max_w), min_w);
end

%% --- Output Function (Torque Computation) ---
function sys = mdlOutputs(~, x, u)
    global node c_M c_C c_G b scale_e scale_de Fai

    qd = u(1:6); dqd = u(7:12); ddqd = u(13:18);
    q = u(19:24); dq = u(25:30);
    I = u(31:36);

    e = qd - q;
    de = dqd - dq;

    % Normalize errors
    en = e ./ scale_e';
    den = de ./ scale_de';

    % Sliding surface and normalize
    r = de + Fai * e;
    scale_r = [1 1 1 0.8 0.7 0.6]';
    r_norm = r ./ scale_r;

    % RBF basis for M, C, G
    h_M = zeros(node,6);
    h_C = zeros(node,6);
    h_G = zeros(node,6);

    for i=1:6
        for j=1:node
            z = [en(i); den(i)];
            c_vec = [c_M(j); c_M(j)];
            h_M(j,i) = exp(-norm(z - c_vec)^2 / b^2);
            h_C(j,i) = exp(-norm(z - c_vec)^2 / b^2);
            h_G(j,i) = exp(-norm(z - c_vec)^2 / b^2);
        end
    end

    % Extract weights
    W_M = reshape(x(1:6*node), node, 6)';
    W_C = reshape(x(6*node+1:12*node), node, 6)';
    W_G = reshape(x(12*node+1:end), node, 6)';

    % Approximate dynamics terms
    M_hat = zeros(6,1);
    C_hat = zeros(6,1);
    G_hat = zeros(6,1);
    for i=1:6
        M_hat(i) = W_M(i,:) * h_M(:,i);
        C_hat(i) = W_C(i,:) * h_C(:,i);
        G_hat(i) = W_G(i,:) * h_G(:,i);
    end

    % Gains
    Kr = 0.05 * eye(6);
    Kp = 3 * eye(6);
    Ki = 1 * eye(6);

    phi = 0.01;  % boundary layer thickness
    sat_r = max(min(r_norm/phi, 1), -1);

    % Torque computation
    tol_m = M_hat .* ddqd + C_hat .* dqd + G_hat;
    tol_r = Kr * sat_r;
    tau = tol_m + Kp * r + Ki * I + tol_r;

    % Torque saturation limits
    tau_max = 50; tau_min = -50;
    tau = max(min(tau, tau_max), tau_min);

    % Clean output
    tau(~isfinite(tau)) = 0;
    tau = real(tau(:));

    if length(tau) ~= 6
        error('Output torque vector tau must be length 6');
    end

    sys = tau;
    
end



