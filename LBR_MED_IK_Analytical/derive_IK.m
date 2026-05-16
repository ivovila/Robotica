syms q1 q2 q3 q4 q5 q6 q7 real
Robot = [0.340,   q1,     0,    pi/2,   0;
         0,       q2,     0,   -pi/2,   0;
         0.400,   q3,     0,   -pi/2,   0;
         0,       q4,     0,    pi/2,   0;
         0.400,   q5,     0,    pi/2,   0;
         0,       q6,     0,   -pi/2,   0;
         0.126,   q7,     0,    0,      0];

Robot_sub = subs(Robot, q3, 0);

T01 = DHTransf(Robot_sub(1,:));
T12 = DHTransf(Robot_sub(2,:));
T23 = DHTransf(Robot_sub(3,:));
T34 = DHTransf(Robot_sub(4,:));
T45 = DHTransf(Robot_sub(5,:));

T05 = simplify(T01*T12*T23*T34*T45);
pw = T05(1:3, 4);

fprintf('Wrist center position pw (with q3=0):\n');
disp(pw);
