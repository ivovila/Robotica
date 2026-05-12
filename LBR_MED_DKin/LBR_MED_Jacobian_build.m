%% LBR_MED_Jacobian_build  Generate Simulink library and simulation model for Jacobian
%
%  This script follows the style of the teacher's example (LBR_MED_build.m)
%  to create a Simulink block for the geometric Jacobian and a test model.

%% 1. Setup Paths
toolboxPath = fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d');
addpath(toolboxPath);

%% 2. Symbolic Geometric Jacobian Derivation
% Formula from class: 
% J_Pi = z_{i-1} x (p_e - p_{i-1})
% J_Oi = z_{i-1}
fprintf('Computing symbolic geometric Jacobian...\n');

Robot = LBR_MED();
n = size(Robot,1);
vars = symvar(Robot); % [q1 q2 q3 q4 q5 q6 q7]

% Get intermediate transformation matrices
T_mats = cell(n, 1);
T_accum = eye(4);
for i = 1:n
    T_accum = T_accum * DHTransf(Robot(i,:));
    T_mats{i} = simplify(T_accum);
end

pe = T_mats{n}(1:3, 4); % End-effector position
J = sym(zeros(6, n));

% Column 1 (Base frame 0: z0=[0;0;1], p0=[0;0;0])
z0 = [0; 0; 1];
p0 = [0; 0; 0];
J(1:3, 1) = cross(z0, pe - p0);
J(4:6, 1) = z0;

% Columns 2 to n
for i = 2:n
    zi_1 = T_mats{i-1}(1:3, 3);
    pi_1 = T_mats{i-1}(1:3, 4);
    J(1:3, i) = cross(zi_1, pe - pi_1);
    J(4:6, i) = zi_1;
end

J = simplify(J);
fprintf('Done.\n\n');

%% 3. Create Simulink Library
libName = 'LBR_MED_Jacobian_Lib';
if bdIsLoaded(libName), close_system(libName, 0); end
new_system(libName, 'Library');
open_system(libName);

% Create the Jacobian block
% We use 'Vars' to ensure q7 is included even if mathematically eliminated
matlabFunctionBlock([libName '/LBR_MED_Jacobian'], J, 'Vars', vars);

save_system(libName);
close_system(libName);
fprintf('Created %s.slx\n', libName);

%% 4. Create Simulation Model
mdlName = 'LBR_MED_Jacobian_Simul';
if bdIsLoaded(mdlName), close_system(mdlName, 0); end
new_system(mdlName);
open_system(mdlName);

% Add Jacobian block from library (or recreate it)
jBlk = [mdlName '/LBR_MED_Jacobian'];
matlabFunctionBlock(jBlk, J, 'Vars', vars);
set_param(jBlk, 'Position', [300 100 500 450]);

% Add joint inputs (Constant blocks)
for i = 1:n
    cBlk = sprintf('%s/q%d', mdlName, i);
    add_block('simulink/Sources/Constant', cBlk, ...
        'Value', '0', 'Position', [60, 40 + (i-1)*60, 140, 70 + (i-1)*60]);
    
    % Connect to Jacobian block
    ph_src = get_param(cBlk, 'PortHandles');
    ph_dst = get_param(jBlk, 'PortHandles');
    Simulink.connectBlocks(ph_src.Outport(1), ph_dst.Inport(i));
end

% Add Display for the Jacobian Matrix
dspJ = [mdlName '/Display_Jacobian'];
add_block('simulink/Sinks/Display', dspJ, 'Position', [600 100 850 450]);
ph_j = get_param(jBlk, 'PortHandles');
ph_dspJ = get_param(dspJ, 'PortHandles');
Simulink.connectBlocks(ph_j.Outport(1), ph_dspJ.Inport(1));

% Add Rank Calculation using a simple Interpreted MATLAB Function
% This is the most compatible way to run 'rank()' on a matrix in Simulink
rankBlk = [mdlName '/Rank_Calculation'];
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', rankBlk, ...
    'Position', [600 500 700 530]);
set_param(rankBlk, 'MATLABFcn', 'rank');

% Display for the Rank
dspR = [mdlName '/Display_Rank'];
add_block('simulink/Sinks/Display', dspR, 'Position', [750 500 800 530]);

% Connect Jacobian -> Rank -> Display
ph_rank = get_param(rankBlk, 'PortHandles');
ph_dspR = get_param(dspR, 'PortHandles');
Simulink.connectBlocks(ph_j.Outport(1), ph_rank.Inport(1));
Simulink.connectBlocks(ph_rank.Outport(1), ph_dspR.Inport(1));

save_system(mdlName);
fprintf('Created %s.slx\n', mdlName);
fprintf('To show singularities: set q6=0 and check the Rank Display.\n');
