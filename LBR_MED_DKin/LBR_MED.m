function Robot = LBR_MED()
% DH table for KUKA LBR Med
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
