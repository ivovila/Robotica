%% LBR_MED_validate  Validate direct kinematics for KUKA LBR Med 7 R800
%
%  Four configurations whose end-effector pose can be determined
%  analytically from the DH table without symbolic computation:
%
%  Config 1: q = [0 0 0 0 0 0 0]  (home – arm fully extended upward)
%    d1+d3+d5+d7 = 0.340+0.400+0.400+0.126 = 1.266 m along world z
%    → p_exp = [0, 0, 1.266]^T        R_exp = I
%
%  Config 2: q = [0 pi/2 0 0 0 0 0]  (shoulder bend 90 deg)
%    Shoulder at height d1 = 0.340 m; remaining links extend
%    horizontally in -x direction (reach = d3+d5+d7 = 0.926 m)
%    → p_exp = [-0.926, 0, 0.340]^T   R_exp = Ry(-pi/2)
%
%  Config 3: q = [pi/2 0 0 0 0 0 0]  (base spin 90 deg)
%    Base rotation does not move the tip of a vertical arm
%    → p_exp = [0, 0, 1.266]^T        R_exp = Rz(pi/2)
%
%  Config 4: q = [0 pi/2 0 pi/2 0 0 0] (zigzag bend)
%    Shoulder tilts forward, elbow tilts back; tip stays closer to base.
%    → p_exp = [-0.400, 0, 0.866]^T   R_exp = I

toolboxPath = fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d');
addpath(toolboxPath);

fprintf('Building symbolic direct kinematics...\n');
Robot = LBR_MED();
T_sym = DKin(Robot);
vars  = symvar(Robot);   % [q1 q2 q3 q4 q5 q6 q7] alphabetical = numerical order
fprintf('Done.\n\n');

eval_T = @(qs) double(subs(T_sym, vars, qs));

% ---- Test cases: {joint angles, expected R, expected p, description} ----
I = eye(3);
Rz90 = [0 -1 0; 1 0 0; 0 0 1];
Ry_90 = [0 0 -1; 0 1 0; 1 0 0];

tests = {
    [0,    0,    0, 0, 0, 0, 0],    I,      [0;       0; 1.266],  'home – arm vertical';
    [0,    pi/2, 0, 0, 0, 0, 0],    Ry_90,  [-0.926;  0; 0.340],  'shoulder bend +90 deg';
    [pi/2, 0,    0, 0, 0, 0, 0],    Rz90,   [0;       0; 1.266],  'base spin +90 deg';
    [0,    pi/2, 0, pi/2, 0, 0, 0], I,      [-0.400;  0; 0.866],  'zigzag (q2=q4=pi/2)';
};

fprintf('=== KUKA LBR Med 7 R800 – Direct Kinematics Validation ===\n\n');

tol = 1e-6;
allPass = true;

for k = 1:size(tests,1)
    q_test = tests{k,1};
    R_exp  = tests{k,2};
    p_exp  = tests{k,3};
    desc   = tests{k,4};

    T   = eval_T(q_test);
    p   = T(1:3, 4);
    R   = T(1:3, 1:3);
    
    err_p = norm(p - p_exp);
    err_R = norm(R - R_exp, 'fro');
    
    ok_p = (err_p < tol);
    ok_R = (err_R < tol);
    ok   = ok_p && ok_R;
    allPass = allPass && ok;

    if ok, status = 'PASS'; else, status = 'FAIL'; end

    fprintf('Config %d: %s\n', k, desc);
    fprintf('  q      = [%s] rad\n', num2str(q_test, '%.4f '));
    fprintf('  p_exp  = [%7.4f  %7.4f  %7.4f]^T m\n', p_exp);
    fprintf('  p_got  = [%7.4f  %7.4f  %7.4f]^T m   err_p = %.2e\n\n', ...
            p(1), p(2), p(3), err_p);
    fprintf('  R_exp  = [%7.4f  %7.4f  %7.4f;\n', R_exp(1,:));
    fprintf('           [%7.4f  %7.4f  %7.4f;\n', R_exp(2,:));
    fprintf('           [%7.4f  %7.4f  %7.4f;]\n\n', R_exp(3,:));
    fprintf('  R_got  = [%7.4f  %7.4f  %7.4f;\n', R(1,:));
    fprintf('           [%7.4f  %7.4f  %7.4f;\n', R(2,:));
    fprintf('           [%7.4f  %7.4f  %7.4f;]\n\n', R(3,:));
    fprintf('  Result: %s (err_R = %.2e)\n', status, err_R);
    fprintf('\n');
end

if allPass
    fprintf('All %d validation tests PASSED (Position & Orientation).\n', size(tests,1));
else
    fprintf('One or more tests FAILED – review DH table in LBR_MED.m.\n');
end

