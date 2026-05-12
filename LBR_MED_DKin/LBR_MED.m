function Robot = LBR_MED()
%LBR_MED  DH table for KUKA LBR Med 7 R800 (7-DOF surgical manipulator)
%
%   Format (per toolbox convention): [d, theta, a, alpha, offset]
%   All joints are revolute; theta_i = q_i (symbolic).
%
%   Standard DH parameters:
%   Link   d(m)   theta   a(m)   alpha(rad)   Joint
%    1    0.340    q1      0      +pi/2    Base rotation
%    2    0        q2      0      -pi/2    Shoulder tilt
%    3    0.400    q3      0      -pi/2    Upper-arm rotation
%    4    0        q4      0      +pi/2    Elbow tilt
%    5    0.400    q5      0      +pi/2    Forearm rotation
%    6    0        q6      0      -pi/2    Wrist tilt
%    7    0.126    q7      0       0       End-effector rotation

syms q1 q2 q3 q4 q5 q6 q7 real

%        d        theta   a     alpha    offset
Robot = [0.340,   q1,     0,    pi/2,   0;
         0,       q2,     0,   -pi/2,   0;
         0.400,   q3,     0,   -pi/2,   0;
         0,       q4,     0,    pi/2,   0;
         0.400,   q5,     0,    pi/2,   0;
         0,       q6,     0,   -pi/2,   0;
         0.126,   q7,     0,    0,      0];
end
