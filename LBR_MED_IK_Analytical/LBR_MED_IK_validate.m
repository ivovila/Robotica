addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'LBR_MED_DKin'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Robotics_Symbolic_Matlab_Toolbox-2'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d'));

fprintf('=== KUKA LBR Med 7 Analytical IK Validation ===\n\n');

tests = {
    [0, 0, 0, 0, 0, 0, 0], 'Home position';
    [0.2, -0.3, 0, 0.5, 0.1, -0.4, 0.2], 'Random config (q3=0)';
    [pi/4, pi/3, 0, -pi/6, 0.2, 0.5, -0.3], 'Another random config (q3=0)';
};

Robot = LBR_MED();
sym_vars = symvar(Robot);

tol = 1e-6;
all_pass = true;

for i = 1:size(tests, 1)
    q_orig = tests{i,1}';
    desc = tests{i,2};
    
    % Forward Kinematics
    T_orig = eye(4);
    for j = 1:7
        T_orig = T_orig * DHTransf_local(Robot(j,:), q_orig(j));
    end
    p_des = double(T_orig(1:3, 4));
    R_des = double(T_orig(1:3, 1:3));
    
    fprintf('Testing %s...\n', desc);
    fprintf('  p_des: [%.4f, %.4f, %.4f]\n', p_des);
    
    % Inverse Kinematics
    try
        q_ik = LBR_MED_IK(p_des, R_des, q_orig);
        q_ik = double(q_ik);
        
        % verify IK result with FK
        T_ik = eye(4);
        for j = 1:7
            T_ik = T_ik * DHTransf_local(Robot(j,:), q_ik(j));
        end
        p_ik = double(T_ik(1:3, 4));
        R_ik = double(T_ik(1:3, 1:3));
        
        err_p = norm(p_ik - p_des);
        err_R = norm(R_ik - R_des, 'fro');
        
        if err_p < tol && err_R < tol
            fprintf('  PASS (err_p = %.2e, err_R = %.2e)\n', err_p, err_R);
        else
            fprintf('  FAIL (err_p = %.2e, err_R = %.2e)\n', err_p, err_R);
            all_pass = false;
        end
        fprintf('  q_orig: [%s]\n', num2str(q_orig', '%.4f '));
        fprintf('  q_ik:   [%s]\n', num2str(q_ik', '%.4f '));
    catch ME
        fprintf('  ERROR: %s\n', ME.message);
        all_pass = false;
    end
    fprintf('\n');
end

if all_pass
    fprintf('All analytical IK validation tests PASSED.\n');
else
    fprintf('Some IK validation tests FAILED.\n');
end

function A = DHTransf_local(p, qi)
    d = p(1);
    theta = qi + p(5);
    a = p(3);
    alpha = p(4);
    
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,   sa,     ca,    d   ;
         0,   0,      0,     1   ];
end
