%% Toolbox path
toolboxPath = fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d');
addpath(toolboxPath);

%% ---- Symbolic direct kinematics ----
fprintf('Computing symbolic direct kinematics\n');
Robot  = LBR_MED();
T      = DKin(Robot);
R_sym  = T(1:3, 1:3);
p_sym  = T(1:3, 4);
fprintf('Done.\n\n');

vars = symvar(Robot);

%% ---- Simulink Library ----
libName = 'LBR_MED_Lib';
if bdIsLoaded(libName), close_system(libName, 0); end
new_system(libName, 'Library');
open_system(libName);

% Embedded MATLAB function block for DK
matlabFunctionBlock([libName '/LBR_MED_DirectKinematics'], R_sym, p_sym);

% Optional Sim3D visualisation (requires Simulink 3D Animation toolbox)
try
    frameWrl = fullfile(toolboxPath, 'frame_axes.wrl');
    buildDHActors(Robot, libName, frameWrl);
    fprintf('3D visualisation actors added to library.\n');
catch ME
    warning('3D visualisation skipped (%s).', ME.message);
end



save_system(libName);
close_system(libName);
fprintf('Created %s.slx\n', libName);

%% ---- Simulink Simulation Model ----
mdlName = 'LBR_MED_Simul';
if bdIsLoaded(mdlName), close_system(mdlName, 0); end
new_system(mdlName);
open_system(mdlName);

% DK function block
dkBlk = [mdlName '/LBR_MED_DirectKinematics'];
matlabFunctionBlock(dkBlk, R_sym, p_sym);
set_param(dkBlk, 'Position', [280 60 530 480]);

% Constant blocks for each joint angle
nJoints = 7;
for i = 1:nJoints
    cBlk = sprintf('%s/q%d', mdlName, i);
    add_block('simulink/Sources/Constant', cBlk, ...
        'Value',           '0', ...
        'OutDataTypeStr',  'double', ...
        'Position', [60, 40 + (i-1)*65, 140, 70 + (i-1)*65]);
    ph_src = get_param(cBlk, 'PortHandles');
    ph_dst = get_param(dkBlk, 'PortHandles');
    Simulink.connectBlocks(ph_src.Outport(1), ph_dst.Inport(i));
end

% Display blocks for p and R
dspP = [mdlName '/Display_p'];
dspR = [mdlName '/Display_R'];
add_block('simulink/Sinks/Display', dspR, 'Position', [620  60 800 200]);
add_block('simulink/Sinks/Display', dspP, 'Position', [620 260 800 340]);

ph_dk = get_param(dkBlk, 'PortHandles');
ph_R  = get_param(dspR,  'PortHandles');
ph_p  = get_param(dspP,  'PortHandles');
Simulink.connectBlocks(ph_dk.Outport(1), ph_R.Inport(1));
Simulink.connectBlocks(ph_dk.Outport(2), ph_p.Inport(1));

save_system(mdlName);
fprintf('Created %s.slx\n\n', mdlName);
fprintf('Usage:\n');
fprintf('  1. Open LBR_MED_Simul.slx\n');
fprintf('  2. Double-click any q_i Constant block to change its value (radians)\n');
fprintf('  3. Run simulation (Ctrl+T) — Display blocks show R and p\n');
fprintf('  4. See LBR_MED_validate.m for expected values at test configurations\n');
