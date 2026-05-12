%% Master_Validation.m
% Comprehensive validation of KUKA LBR Med 7 Kinematics (Points 1-5)

clear; clc; close all;

% --- 0. Pre-check: Close open models to avoid conflicts ---
models = {'LBR_MED_Simul', 'LBR_MED_Jacobian_Simul', 'LBR_MED_IK_Simul', 'LBR_MED_CLIK_Simul'};
for i = 1:length(models)
    if bdIsLoaded(models{i}), close_system(models{i}, 0); end
end

% --- 1. Path Setup ---
fprintf('--- Setting up paths ---\n');
project_root = pwd;
% Remove old paths if they exist to avoid shadowing
warning('off', 'MATLAB:rmpath:DirNotFound');
folders = {'LBR_MED_DKin', 'LBR_MED_IK_Analytical', 'LBR_MED_CLIK', 'Robotics_Symbolic_Matlab_Toolbox-2', 'RobotX_sim3d'};
for i = 1:length(folders)
    f_path = fullfile(project_root, folders{i});
    rmpath(f_path); % Try to remove first
    if exist(f_path, 'dir')
        addpath(f_path);
    else
        warning('Directory not found: %s', f_path);
    end
end
warning('on', 'MATLAB:rmpath:DirNotFound');

% Clear MATLAB's internal caches
clear functions;
rehash;

%% --- 2. Direct Kinematics Validation (Point 2) ---
fprintf('\n--- Validating Point 2: Direct Kinematics ---\n');
run('LBR_MED_validate.m');

%% --- 3. Geometric Jacobian Validation (Point 4) ---
fprintf('\n--- Validating Point 4: Geometric Jacobian ---\n');
run('LBR_MED_validate_Jacobian.m');

%% --- 4. Analytical Inverse Kinematics Validation (Point 3) ---
fprintf('\n--- Validating Point 3: Analytical Inverse Kinematics ---\n');
run('LBR_MED_IK_validate.m');

%% --- 5. CLIK and Point 3 Cross-Validation (Point 5) ---
fprintf('\n--- Validating Point 5: Closed-Loop Inverse Kinematics ---\n');
% Ensure we are in the root when running the setup
cd(project_root);
run('LBR_MED_CLIK_setup.m');

fprintf('\n=== Master Validation Complete ===\n');
fprintf('Review the command window output for PASS/FAIL status of each component.\n');
fprintf('Review the CLIK plot for joint convergence.\n');
fprintf('The Simulink model "LBR_MED_CLIK_Simul" is now open and ready for interactive testing.\n');
