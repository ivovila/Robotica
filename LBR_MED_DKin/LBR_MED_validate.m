%% LBR_MED_validate  Validate direct kinematics for KUKA LBR Med 7 R800
%
%  Three configurations whose end-effector pose can be determined
%  analytically from the DH table without symbolic computation:
%
%  Config 1: q = [0 0 0 0 0 0 0]  (home – arm fully extended upward)
%    d1+d3+d5+d7 = 0.340+0.400+0.400+0.126 = 1.266 m along world z
%    → p_exp = [0, 0, 1.266]^T        R_exp = I_3
%
%  Config 2: q = [pi/2 0 0 0 0 0 0]  (only base joint rotated 90 deg)
%    Base rotation does not move the tip of a vertical arm
%    → p_exp = [0, 0, 1.266]^T        R_exp = Rz(pi/2)
%
%  Config 3: q = [0 pi/2 0 0 0 0 0]  (shoulder tilted 90 deg)
%    Shoulder at height d1 = 0.340 m; remaining links extend
%    horizontally in -x direction (reach = d3+d5+d7 = 0.926 m)
%    → p_exp = [-0.926, 0, 0.340]^T

toolboxPath = fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d');
addpath(toolboxPath);

fprintf('Building symbolic direct kinematics...\n');
Robot = LBR_MED();
T_sym = DKin(Robot);
vars  = symvar(Robot);   % [q1 q2 q3 q4 q5 q6 q7] alphabetical = numerical order
fprintf('Done.\n\n');

eval_T = @(qs) double(subs(T_sym, vars, qs));

% ---- Test cases: {joint angles, expected position (column vector)} ----
tests = {
    [0,      0, 0, 0, 0, 0, 0],  [0;       0; 1.266],  'home – arm vertical';
    [pi/2,   0, 0, 0, 0, 0, 0],  [0;       0; 1.266],  'base +90 deg, position unchanged';
    [0,   pi/2, 0, 0, 0, 0, 0],  [-0.926;  0; 0.340],  'shoulder +90 deg, arm horizontal (-x)';
};

fprintf('=== KUKA LBR Med 7 R800 – Direct Kinematics Validation ===\n\n');

tol = 1e-6;
allPass = true;

for k = 1:size(tests,1)
    q_test = tests{k,1};
    p_exp  = tests{k,2};
    desc   = tests{k,3};

    T   = eval_T(q_test);
    p   = T(1:3, 4);
    R   = T(1:3, 1:3);
    err = norm(p - p_exp);
    ok  = (err < tol);
    allPass = allPass && ok;

    if ok, status = 'PASS'; else, status = 'FAIL'; end

    fprintf('Config %d: %s\n', k, desc);
    fprintf('  q      = [%s] rad\n', num2str(q_test, '%.4f '));
    fprintf('  p_exp  = [%7.4f  %7.4f  %7.4f]^T m\n', p_exp);
    fprintf('  p_got  = [%7.4f  %7.4f  %7.4f]^T m   ||err|| = %.2e   %s\n', ...
            p(1), p(2), p(3), err, status);
    fprintf('  R_got  =\n');
    for row = 1:3
        fprintf('           [%7.4f  %7.4f  %7.4f]\n', R(row,:));
    end
    fprintf('\n');
end

if allPass
    fprintf('All %d validation tests PASSED.\n', size(tests,1));
else
    fprintf('One or more tests FAILED – review DH table in LBR_MED.m.\n');
end
