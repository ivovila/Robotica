function buildDHActors(DHTable, modelName, frameWrlFile)
% buildDHActors Create a Simulink 3D model with one actor per DH frame
%
% DHTable : DH table of parameters
% modelName    : name of the Simulink model to create
% frameWrlFile : path to the WRL file used to draw a reference frame
%
% Each actor is intended to display one DH reference frame.

if nargin < 1
    error('You must provide DHTable.');
end
if nargin < 2 || isempty(modelName)
    modelName = 'dh_frame_visualization';
    % Close old model if loaded
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    % Create model
    new_system(modelName,'Library');
end
if nargin < 3 || isempty(frameWrlFile)
    frameWrlFile = fullfile(pwd, 'frame_axes.wrl');
end


open_system(modelName);

% Load library
load_system('sim3dlib');
blkList = {};

modelName = [modelName '/Visual'];
add_block('simulink/Ports & Subsystems/Subsystem',modelName);
Simulink.SubSystem.deleteContents(modelName);
% Add Scene Configuration block
sceneBlk = [modelName '/Scene Configuration'];
add_block('sim3dlib/Simulation 3D Scene Configuration', sceneBlk, ...
    'Position', [40 40 240 100]);
set_param(sceneBlk, 'SceneDesc', 'Empty scene')
blkList{end+1} =sceneBlk; 
% Add one actor per frame
x0 = 350;
y0 = 100;
dy = 120;
nFrames = size(DHTable, 1);
scale_blk = [modelName '/Scale'];
add_block('simulink/Sources/Constant', scale_blk , 'Position', [40 100 80 140]);
for i = 0:nFrames
    blk = sprintf('%s/Frame_%d', modelName, i);

    add_block('sim3dlib/Simulation 3D Actor', blk, ...
        'Position', [x0+100 y0+(i-1)*dy x0+220+100 y0+60+(i-1)*dy]);
    

    initScript = [
        "Actor.CoordinateSystem = ""MATLAB"";"
        ];

    set_param(blk, 'InitScriptText', strjoin(initScript, newline));

    blkList{end+1} = blk;


    try
        set_param(blk, 'ActorName', sprintf('Frame_%d', i));
    catch
    end

    try
        set_param(blk, 'Operation', 'Create at setup');
    catch
    end

    try
        set_param(blk, 'SourceFile', frameWrlFile);
    catch
    end

    
    %sprintf('%s/Frame_%d', modelName, i);
    %inputParamsBlk = [sprintf('%s/Frame_%d.Translation', modelName,  i) newline sprintf('%s/Frame_%d.Rotation   ', modelName, i)];
    %set_param(blk, 'InputsText', inputParamsBlk);

    if i > 0
        set_param(blk, 'ParentName', sprintf('Frame_%d', i-1));
        dhi = DHTable(i,:);
        
        [Reul, p] = DHTransf_eul_trl(dhi);
        

        Tblk = [modelName sprintf('/T^%d_%d', i-1, i)]; 
        matlabFunctionBlock(Tblk,p, Reul);
        set_param(Tblk,...
                    Position=[x0-150 y0+(i-1)*dy x0-150+100 y0+60+(i-1)*dy])
        blkList{end+1} =Tblk; 

        blk_transf = sprintf('%s/Transform_Frame_%d', modelName, i);
        add_block('sim3dlib/Simulation 3D Actor Transform Set', blk_transf, ...
            'Position', [x0 y0+(i-1)*dy x0+220 y0+60+(i-1)*dy]);
        set_param(blk_transf, 'CoordinateSystem', 'MATLAB', 'ActorTag', sprintf('Frame_%d', i) )
        
        src = Tblk;
        dst = blk_transf;
        ph_T = get_param(src, 'PortHandles');
        ph_F = get_param(dst, 'PortHandles');
        %add_line(model, ph_T.Outport(1), ph_F.Input(1), 'autorouting', 'on');
        %add_line(model, ph_T.Outport(2), ph_F.Input(2), 'autorouting', 'on');

        connection = Simulink.connectBlocks(ph_T.Outport(1),ph_F.Inport(1));
        connection = Simulink.connectBlocks(ph_T.Outport(2),ph_F.Inport(2));
        ph_sc = get_param(scale_blk, 'PortHandles');

        connection = Simulink.connectBlocks(ph_sc.Outport(1),ph_F.Inport(3));
        
        add_block('simulink/Sources/In1', [modelName sprintf('/q%d',i)], ...
            ... 'Position', [30 30 60 50], ...
            'Position', [x0-200 y0+(i-1)*dy+10 x0-200+15 y0+20+(i-1)*dy+10],...
            'Port', sprintf('%d',i)); % optional: set port number


        set_param([modelName sprintf('/q%d',i)], 'Name', sprintf('q%d',i));   % change display name
        ph_in = get_param([modelName sprintf('/q%d',i)], 'PortHandles');
        Simulink.connectBlocks(ph_in.Outport(1),ph_T.Inport(1));
    end
    
end

%save_system(modelName(1:end-length('/Visual')+1));

end


function [eul, t] = DHTransf_eul_trl( p )
%DHTransf Returns the symbolic D-H joint transformation matrix
% p = [d v a alpha offset]

% Check if it is a rotational or translational joint and add offset
% symbolically.
q=symvar(p);        %get symbolic variable from p
if ~isempty(symvar(p(2)))  %if it is a rotational joint
    p(2)=p(2)+p(5);  %add offset to v
else
    p(1)=p(1)+p(5);  %add offset to d
end

% Build D-H matrix
eul = [p(4) 0 p(2)];
t=[ p(3)*cos(p(2)),   p(3)*sin(p(2)),   p(1)    ];
end

