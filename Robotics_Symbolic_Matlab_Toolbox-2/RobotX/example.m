%% Generate Matlab and Simulink Code for RobotX
%  Create Simulink Library file RobotX_Lib.mdl
new_system('RobotX_Lib','Library');
open_system('RobotX_Lib');

%% Direct Kinematics and Jacobian
[RobotX_T]=DKin(RobotX);
RobotX_R=RobotX_T(1:3,1:3);
RobotX_p=RobotX_T(1:3,4);

%Generate optimized embeded Matlab function blocks for Simulink
matlabFunctionBlock('RobotX_Lib/RobotX_Direct_Kinematics',RobotX_R,RobotX_p);

%% Save library in current Directory
save_system('RobotX_Lib');
close_system('RobotX_Lib');
