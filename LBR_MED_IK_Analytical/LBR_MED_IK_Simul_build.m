addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'LBR_MED_DKin'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Robotics_Symbolic_Matlab_Toolbox-2'));
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'RobotX_sim3d'));

mdlName = 'LBR_MED_IK_Simul';
if bdIsLoaded(mdlName), close_system(mdlName, 0); end
new_system(mdlName);
open_system(mdlName);

p_des = [-0.926; 0; 0.340];
R_des = [0 0 -1; 0 1 0; 1 0 0];

add_block('simulink/Sources/Constant', [mdlName '/p_des'], 'Value', mat2str(p_des), 'Position', [50, 50, 150, 80]);
add_block('simulink/Sources/Constant', [mdlName '/R_des'], 'Value', mat2str(R_des), 'Position', [50, 100, 150, 150]);

% IK Block
ikFunc = [
'function q = ik_block(p_des, R_des)\n', ...
'    q = LBR_MED_IK(p_des, R_des);\n', ...
'end'
];
ikBlk = [mdlName '/Analytical_IK'];
add_block('simulink/User-Defined Functions/MATLAB Function', ikBlk, 'Position', [250, 50, 400, 150]);
sf = sfroot;
block = sf.find('Path', ikBlk, '-isa', 'Stateflow.EMChart');
block.Script = sprintf(ikFunc);

% DK Block
Robot = LBR_MED();
T = DKin(Robot);
R_sym = T(1:3, 1:3);
p_sym = T(1:3, 4);
vars = symvar(Robot);

dkBlk = [mdlName '/DirectKinematics'];
matlabFunctionBlock(dkBlk, R_sym, p_sym, 'Vars', vars);
set_param(dkBlk, 'Position', [500, 50, 650, 150]);

Simulink.connectBlocks([mdlName '/p_des'], [ikBlk '/1']);
Simulink.connectBlocks([mdlName '/R_des'], [ikBlk '/2']);

dmx = [mdlName '/Demux_q'];
add_block('simulink/Signal Routing/Demux', dmx, 'Outputs', '7', 'Position', [430, 50, 440, 150]);
Simulink.connectBlocks([ikBlk '/1'], [dmx '/1']);
for i = 1:7
    ph_dmx = get_param(dmx, 'PortHandles');
    ph_dk  = get_param(dkBlk, 'PortHandles');
    Simulink.connectBlocks(ph_dmx.Outport(i), ph_dk.Inport(i));
end

add_block('simulink/Sinks/Display', [mdlName '/Display_p_got'], 'Position', [750, 100, 900, 180]);
add_block('simulink/Sinks/Display', [mdlName '/Display_R_got'], 'Position', [750, 20, 950, 90]);
add_block('simulink/Sinks/Display', [mdlName '/Display_q'], 'Position', [500, 200, 650, 350]);

Simulink.connectBlocks([dkBlk '/2'], [mdlName '/Display_p_got/1']); % p output
Simulink.connectBlocks([dkBlk '/1'], [mdlName '/Display_R_got/1']); % R output
Simulink.connectBlocks([ikBlk '/1'], [mdlName '/Display_q/1']);

save_system(mdlName);
fprintf('Model %s.slx created for Point 3 validation.\n', mdlName);
