%% derive_IK.m
% Symbolic derivation of analytical IK for KUKA LBR Med 7
% Fixing q3 = 0 to reduce to 6-DOF problem

syms q1 q2 q3 q4 q5 q6 q7 real
Robot = [0.340,   q1,     0,    pi/2,   0;
         0,       q2,     0,   -pi/2,   0;
         0.400,   q3,     0,   -pi/2,   0;
         0,       q4,     0,    pi/2,   0;
         0.400,   q5,     0,    pi/2,   0;
         0,       q6,     0,   -pi/2,   0;
         0.126,   q7,     0,    0,      0];

% Fix q3 = 0
Robot_sub = subs(Robot, q3, 0);

% Direct Kinematics to find pw (wrist center)
% pw is the origin of frame 5 (and 6)
T01 = DHTransf(Robot_sub(1,:));
T12 = DHTransf(Robot_sub(2,:));
T23 = DHTransf(Robot_sub(3,:));
T34 = DHTransf(Robot_sub(4,:));
T45 = DHTransf(Robot_sub(5,:));

T05 = simplify(T01*T12*T23*T34*T45);
pw = T05(1:3, 4);

fprintf('Wrist center position pw (with q3=0):\n');
disp(pw);

% Let's see the components
% pw_x = cos(q1)*sin(q2 + q4)*4/10 + cos(q1)*sin(q2)*4/10
% pw_y = sin(q1)*sin(q2 + q4)*4/10 + sin(q1)*sin(q2)*4/10
% pw_z = cos(q2 + q4)*4/10 + cos(q2)*4/10 + 17/50
