%% LBR_MED_Jacobian_validate  Validate geometric Jacobian for KUKA LBR Med 7
%
%  This script compares the analytical (symbolic) Jacobian with a
%  numerical approximation derived from finite differences of the 
%  direct kinematics.

toolboxPath = fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d');
addpath(toolboxPath);

fprintf('Building symbolic Jacobian (may take a moment)...\n');
J_sym = LBR_MED_Jacobian_symbolic();
T_sym = DKin(LBR_MED());
vars  = symvar(LBR_MED());
fprintf('Done.\n\n');

% Function to evaluate T and J at a given configuration
eval_T = @(qs) double(subs(T_sym, vars, qs));
eval_J = @(qs) double(subs(J_sym, vars, qs));

% Numerical Jacobian function
function Jn = numerical_jacobian(q, eval_T_func)
    dq = 1e-6;
    n = length(q);
    Jn = zeros(6, n);
    T0 = eval_T_func(q);
    p0 = T0(1:3, 4);
    R0 = T0(1:3, 1:3);
    
    for i = 1:n
        qi = q;
        qi(i) = qi(i) + dq;
        Ti = eval_T_func(qi);
        pi = Ti(1:3, 4);
        Ri = Ti(1:3, 1:3);
        
        % Linear part: dp/dqi
        Jn(1:3, i) = (pi - p0) / dq;
        
        % Angular part: extraction from S(w) = dR/dqi * R'
        Sw = (Ri - R0) / dq * R0';
        % w = [S(3,2); S(1,3); S(2,1)]
        Jn(4:6, i) = [Sw(3,2); Sw(1,3); Sw(2,1)];
    end
end

% Test configurations
tests = {
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], 'Random posture (Full Rank)';
    [0, 0, 0, 0, 0, 0, 0], 'Home position (Shoulder/Wrist alignment)';
    [0, pi/4, 0, 0, 0, 0, 0], 'Wrist singularity (q6=0 aligns z5 and z7)';
    [0, 0, 0, pi/2, 0, 0, 0], 'Another wrist singularity';
    [0, pi/2, 0, 0, 0, pi/2, 0], 'Elbow singularity (arm fully extended/aligned)';
};

fprintf('=== KUKA LBR Med 7 – Jacobian Validation & Singularity Analysis ===\n\n');

for k = 1:size(tests, 1)
    q_test = tests{k, 1};
    desc = tests{k, 2};
    
    Ja = eval_J(q_test);
    Jn = numerical_jacobian(q_test, eval_T);
    
    err = norm(Ja - Jn, 'fro');
    rk = rank(Ja);
    s = svd(Ja);
    cond_num = max(s)/min(s);
    
    fprintf('Config %d: %s\n', k, desc);
    fprintf('  q          = [%s] rad\n', num2str(q_test, '%.2f '));
    fprintf('  Rank       = %d (n=7 DOF)\n', rk);
    fprintf('  Cond Num   = %.2e\n', cond_num);
    fprintf('  Error (Num) = %.2e\n', err);
    
    if rk < 6
        fprintf('  >>> STATUS: SINGULAR CONFIGURATION (Rank < 6) <<<\n');
        [~, V] = sorted_svd(Ja); % Optional: show null space
    else
        fprintf('  >>> STATUS: FULL RANK <<<\n');
    end
    fprintf('\n');
end

function [U, S, V] = sorted_svd(A)
    [U, S, V] = svd(A);
end
