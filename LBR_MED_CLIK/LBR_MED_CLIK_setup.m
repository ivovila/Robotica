this_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(this_dir);
addpath(fullfile(project_root, 'LBR_MED_DKin'));
addpath(fullfile(project_root, 'Robotics_Symbolic_Matlab_Toolbox-2'));
addpath(fullfile(project_root, 'RobotX_sim3d'));
addpath(fullfile(project_root, 'LBR_MED_IK_Analytical'));

% clear enviroment
mdlName = 'LBR_MED_CLIK_Simul';
if bdIsLoaded(mdlName), close_system(mdlName, 0); end
clear functions;
eval(['clear ', mdlName]);

fprintf('Initializing symbolic models...\n');
Robot = LBR_MED();
n = size(Robot,1);
vars = symvar(Robot);

% symbolic DK
T_sym = DKin(Robot);
R_sym = T_sym(1:3, 1:3);
p_sym = T_sym(1:3, 4);

% symbolic Geometric Jacobian
J = sym(zeros(6, n));
T_mats = cell(n, 1);
T_accum = eye(4);
for i = 1:n
    T_accum = T_accum * DHTransf(Robot(i,:));
    T_mats{i} = simplify(T_accum);
end
pe = T_mats{n}(1:3, 4);
z0 = [0; 0; 1];
p0 = [0; 0; 0];
J(1:3, 1) = cross(z0, pe - p0);
J(4:6, 1) = z0;
for i = 2:n
    zi_1 = T_mats{i-1}(1:3, 3);
    pi_1 = T_mats{i-1}(1:3, 4);
    J(1:3, i) = cross(zi_1, pe - pi_1);
    J(4:6, i) = zi_1;
end
J = simplify(J);

q_vec = sym('q', [n, 1], 'real');
R_sym_v = subs(R_sym, vars.', q_vec);
p_sym_v = subs(p_sym, vars.', q_vec);
J_v = subs(J, vars.', q_vec);

% Simulink Model
mdlName = 'LBR_MED_CLIK_Simul';
cd(this_dir);

fprintf('Saving symbolic models to MAT file for Simulink...\n');
save('LBR_MED_symbolic.mat', 'R_sym', 'p_sym', 'J', 'vars');

if bdIsLoaded(mdlName), close_system(mdlName, 0); end
new_system(mdlName);
open_system(mdlName);

K_pos = 10;
K_ori = 10;
Kn_val = 5; 
q_rest = zeros(n, 1);
q0 = [0, 0.2, 0, -0.2, 0, 0.2, 0]';

p_des = [-0.926; 0; 0.340];
R_des = [0 0 -1; 0 1 0; 1 0 0]; 

add_block('simulink/Sources/Constant', [mdlName '/p_des'], 'Value', mat2str(p_des), 'Position', [50, 50, 150, 80]);
add_block('simulink/Sources/Constant', [mdlName '/R_des'], 'Value', mat2str(R_des), 'Position', [50, 100, 150, 150]);
add_block('simulink/Sources/Constant', [mdlName '/q_rest'], 'Value', mat2str(q_rest), 'Position', [50, 200, 150, 230]);

add_block('simulink/Sources/Constant', [mdlName '/K_pos'], 'Value', num2str(K_pos), 'Position', [50, 250, 150, 280]);
add_block('simulink/Sources/Constant', [mdlName '/K_ori'], 'Value', num2str(K_ori), 'Position', [50, 300, 150, 330]);
add_block('simulink/Sources/Constant', [mdlName '/Kn_val'], 'Value', num2str(Kn_val), 'Position', [50, 350, 150, 380]);

clikFunc = 'function q_dot = clik_controller(p_des, R_des, p_curr, R_curr, J, q_curr, q_rest, K_pos, K_ori, Kn_val)\n    q_dot = clik_controller_logic(p_des, R_des, p_curr, R_curr, J, q_curr, q_rest, K_pos, K_ori, Kn_val);\nend';

ctrlBlk = [mdlName '/CLIK_Controller'];
if ~isempty(find_system(mdlName, 'Name', 'CLIK_Controller'))
    delete_block(ctrlBlk);
end
add_block('simulink/User-Defined Functions/MATLAB Function', ctrlBlk, 'Position', [400, 100, 550, 300]);

rt = sfroot;
block = rt.find('-isa', 'Stateflow.EMChart', 'Path', ctrlBlk);
block.Script = sprintf(clikFunc);

inputs = block.find('-isa', 'Stateflow.Data', 'Scope', 'Input');
for i = 1:length(inputs)
    switch inputs(i).Name
        case {'p_des', 'p_curr'}
            inputs(i).Props.Array.Size = '[3, 1]';
        case {'R_des', 'R_curr'}
            inputs(i).Props.Array.Size = '[3, 3]';
        case 'J'
            inputs(i).Props.Array.Size = '[6, 7]';
        case {'q_curr', 'q_rest'}
            inputs(i).Props.Array.Size = '[7, 1]';
        case {'K_pos', 'K_ori', 'Kn_val'}
            inputs(i).Props.Array.Size = '1';
    end
end

outputs = block.find('-isa', 'Stateflow.Data', 'Scope', 'Output');
for i = 1:length(outputs)
    if strcmp(outputs(i).Name, 'q_dot')
        outputs(i).Props.Array.Size = '[7, 1]';
    end
end
% Forward Kinematics Block
dkBlk = [mdlName '/ForwardKinematics'];
if isempty(find_system(mdlName, 'Name', 'ForwardKinematics'))
    matlabFunctionBlock(dkBlk, R_sym_v, p_sym_v, 'Vars', {q_vec});
end
set_param(dkBlk, 'Position', [700, 300, 850, 400]);

% Jacobian Block
jacBlk = [mdlName '/Jacobian'];
if isempty(find_system(mdlName, 'Name', 'Jacobian'))
    matlabFunctionBlock(jacBlk, J_v, 'Vars', {q_vec});
end
set_param(jacBlk, 'Position', [700, 150, 850, 250]);

% 3D Visualization
try
    if exist('sim3dlib', 'file') || exist('sim3dlib', 'dir')
        fprintf('Adding 3D visualization...\n');
        frameWrl = fullfile(project_root, 'RobotX_sim3d', 'frame_axes.wrl');
        buildDHActors(Robot, mdlName, frameWrl);
        visBlk = [mdlName '/Visual'];
        set_param(visBlk, 'Position', [700, 450, 850, 550]);
        
        ph_int = get_param(intBlk, 'PortHandles');
        demuxVis = [mdlName '/Demux_Vis'];
        if isempty(find_system(mdlName, 'Name', 'Demux_Vis'))
            add_block('simulink/Signal Routing/Demux', demuxVis, 'Outputs', '7', 'Position', [650, 450, 660, 550]);
        end
        Simulink.connectBlocks(ph_int.Outport(1), get_param(demuxVis, 'PortHandles').Inport(1));
        for i = 1:7
            Simulink.connectBlocks([demuxVis '/' num2str(i)], [visBlk '/' num2str(i)]);
        end
    else
        fprintf('3D visualization skipped (Simulink 3D Animation toolbox not found).\n');
    end
catch ME
    fprintf('3D visualization skipped: %s\n', ME.message);
end

intBlk = [mdlName '/Integrator'];
add_block('simulink/Continuous/Integrator', intBlk, 'InitialCondition', mat2str(q0), 'Position', [600, 50, 630, 100]);

ph_int = get_param(intBlk, 'PortHandles');
ph_dk = get_param(dkBlk, 'PortHandles');
ph_jac = get_param(jacBlk, 'PortHandles');
ph_ctrl = get_param(ctrlBlk, 'PortHandles');

Simulink.connectBlocks(ph_int.Outport(1), ph_dk.Inport(1));
Simulink.connectBlocks(ph_int.Outport(1), ph_jac.Inport(1));
Simulink.connectBlocks(ph_int.Outport(1), ph_ctrl.Inport(6)); % q_curr

Simulink.connectBlocks(ph_dk.Outport(1), ph_ctrl.Inport(4)); % R_curr
Simulink.connectBlocks(ph_dk.Outport(2), ph_ctrl.Inport(3)); % p_curr

Simulink.connectBlocks(ph_jac.Outport(1), ph_ctrl.Inport(5)); % J

Simulink.connectBlocks(ph_ctrl.Outport(1), ph_int.Inport(1)); % q_dot

Simulink.connectBlocks([mdlName '/p_des'], [ctrlBlk '/1']);
Simulink.connectBlocks([mdlName '/R_des'], [ctrlBlk '/2']);
Simulink.connectBlocks([mdlName '/q_rest'], [ctrlBlk '/7']);
Simulink.connectBlocks([mdlName '/K_pos'], [ctrlBlk '/8']);
Simulink.connectBlocks([mdlName '/K_ori'], [ctrlBlk '/9']);
Simulink.connectBlocks([mdlName '/Kn_val'], [ctrlBlk '/10']);

dspP = [mdlName '/Display_p_curr'];
dspR = [mdlName '/Display_R_curr'];
if isempty(find_system(mdlName, 'Name', 'Display_p_curr')), add_block('simulink/Sinks/Display', dspP, 'Position', [900, 350, 1050, 430]); end
if isempty(find_system(mdlName, 'Name', 'Display_R_curr')), add_block('simulink/Sinks/Display', dspR, 'Position', [900, 250, 1100, 340]); end
Simulink.connectBlocks(ph_dk.Outport(2), get_param(dspP, 'PortHandles').Inport(1));
Simulink.connectBlocks(ph_dk.Outport(1), get_param(dspR, 'PortHandles').Inport(1));

dspQ = [mdlName '/Display_q_final'];
if isempty(find_system(mdlName, 'Name', 'Display_q_final')), add_block('simulink/Sinks/Display', dspQ, 'Position', [900, 50, 1050, 200]); end
Simulink.connectBlocks(ph_int.Outport(1), get_param(dspQ, 'PortHandles').Inport(1));

errFunc = [
'function [e_p, e_o_norm] = error_calc(p_des, R_des, p_curr, R_curr)\n', ...
'    e_p = norm(p_des - p_curr);\n', ...
'    n_e = R_curr(:,1); s_e = R_curr(:,2); a_e = R_curr(:,3);\n', ...
'    n_d = R_des(:,1); s_d = R_des(:,2); a_d = R_des(:,3);\n', ...
'    e_o = 0.5 * (cross(n_e, n_d) + cross(s_e, s_d) + cross(a_e, a_d));\n', ...
'    e_o_norm = norm(e_o);\n', ...
'end'
];
errBlk = [mdlName '/Error_Monitor'];
if isempty(find_system(mdlName, 'Name', 'Error_Monitor'))
    add_block('simulink/User-Defined Functions/MATLAB Function', errBlk, 'Position', [400, 400, 550, 500]);
    sf = sfroot;
    block = sf.find('Path', errBlk, '-isa', 'Stateflow.EMChart');
    block.Script = sprintf(errFunc);
    
    Simulink.connectBlocks([mdlName '/p_des'], [errBlk '/1']);
    Simulink.connectBlocks([mdlName '/R_des'], [errBlk '/2']);
    Simulink.connectBlocks(ph_dk.Outport(2), [errBlk '/3']); % p_curr
    Simulink.connectBlocks(ph_dk.Outport(1), [errBlk '/4']); % R_curr
    
    dspEP = [mdlName '/Display_PosError'];
    dspEO = [mdlName '/Display_OriError'];
    if isempty(find_system(mdlName, 'Name', 'Display_PosError')), add_block('simulink/Sinks/Display', dspEP, 'Position', [600, 400, 700, 430]); end
    if isempty(find_system(mdlName, 'Name', 'Display_OriError')), add_block('simulink/Sinks/Display', dspEO, 'Position', [600, 460, 700, 490]); end
    Simulink.connectBlocks([errBlk '/1'], [dspEP '/1']);
    Simulink.connectBlocks([errBlk '/2'], [dspEO '/1']);
end

scopeQ = [mdlName '/Scope_q'];
if isempty(find_system(mdlName, 'Name', 'Scope_q')), add_block('simulink/Sinks/Scope', scopeQ, 'Position', [700, 20, 730, 50]); end
Simulink.connectBlocks(ph_int.Outport(1), get_param(scopeQ, 'PortHandles').Inport(1));

if isempty(find_system(mdlName, 'Name', 'ToWS_q'))
    add_block('simulink/Sinks/To Workspace', [mdlName '/ToWS_q'], 'VariableName', 'q_sim', 'SaveFormat', 'Timeseries', 'Position', [700, 70, 750, 90]);
    Simulink.connectBlocks(ph_int.Outport(1), get_param([mdlName '/ToWS_q'], 'PortHandles').Inport(1));
end

if isempty(find_system(mdlName, 'Name', 'ToWS_err'))
    add_block('simulink/Sinks/To Workspace', [mdlName '/ToWS_err'], 'VariableName', 'err_sim', 'SaveFormat', 'Timeseries', 'Position', [700, 520, 750, 540]);
    muxErr = [mdlName '/Mux_Err'];
    if isempty(find_system(mdlName, 'Name', 'Mux_Err'))
        add_block('simulink/Signal Routing/Mux', muxErr, 'Inputs', '2', 'Position', [650, 510, 660, 550]);
    end
    Simulink.connectBlocks([errBlk '/1'], [muxErr '/1']);
    Simulink.connectBlocks([errBlk '/2'], [muxErr '/2']);
    Simulink.connectBlocks(get_param(muxErr, 'PortHandles').Outport(1), get_param([mdlName '/ToWS_err'], 'PortHandles').Inport(1));
end

save_system(mdlName);
fprintf('Model saved in %s. Running simulation...\n', this_dir);
set_param(mdlName, 'StopTime', '5');
simOut = sim(mdlName);

% Plot 1: Joint Positions
q_ts = simOut.find('q_sim');
if ~isempty(q_ts)
    figure('Name', 'Joint Positions');
    data = squeeze(q_ts.Data);
    if size(data, 1) == 7 && size(data, 2) ~= 7, data = data'; end
    plot(q_ts.Time, data, 'LineWidth', 1.5);
    title('Joint Positions during CLIK');
    xlabel('Time (s)'); ylabel('q (rad)');
    legend('q1','q2','q3','q4','q5','q6','q7');
    grid on;
end

% Plot 2: Error
err_ts = simOut.find('err_sim');
if ~isempty(err_ts)
    figure('Name', 'Error Convergence');
    data = squeeze(err_ts.Data);
    if size(data, 1) == 2 && size(data, 2) ~= 2, data = data'; end
    semilogy(err_ts.Time, data(:,1), 'b', 'LineWidth', 1.5); hold on;
    semilogy(err_ts.Time, data(:,2), 'r', 'LineWidth', 1.5);
    title('CLIK Error Convergence (Log Scale)');
    xlabel('Time (s)'); ylabel('Error Norm');
    legend('Position Error (m)', 'Orientation Error');
    grid on;
end

fprintf('CLIK Setup complete.\n');

%% Compare CLIK with Analytical IK
fprintf('\n=== Point 3 Validation: Comparing CLIK with Analytical IK ===\n');

if ~isempty(q_ts)
    if size(q_ts.Data, 2) == 7
        q_clik_final = q_ts.Data(end, :)';
    else
        q_clik_final = q_ts.Data(:, end);
    end
    fprintf('CLIK Steady-State Solution (7-DOF):\n');
    disp(q_clik_final');
    
    try
        q_analytical = LBR_MED_IK(p_des, R_des, q_clik_final);
        fprintf('Analytical IK Solution (Point 3, closest to CLIK):\n');
        disp(q_analytical');
        
        p_clik = double(subs(p_sym_v, q_vec, q_clik_final));
        R_clik = double(subs(R_sym_v, q_vec, q_clik_final));
        
        err_p = norm(p_clik - p_des);
        err_R = norm(R_clik - R_des, 'fro');
        
        fprintf('CLIK Task Space Errors:\n');
        fprintf('  Pos Error: %.2e m\n', err_p);
        fprintf('  Ori Error: %.2e\n', err_R);
        
        err_q = norm(q_clik_final - q_analytical);
        fprintf('Joint space difference from Point 3 analytical solution: %.4e rad\n', err_q);
        
        if err_q < 0.01
             fprintf('>>> VALIDATION PASSED: CLIK matches Analytical IK solution. <<<\n');
        else
             fprintf('>>> VALIDATION NOTE: CLIK and Analytical solutions differ. <<<\n');
        end
    catch ME
        fprintf('Analytical IK failed: %s\n', ME.message);
    end
end

cd(project_root);
