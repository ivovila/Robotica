function [J] = LBR_MED_Jacobian_symbolic()
    % LBR_MED_Jacobian_symbolic  Computes the symbolic geometric Jacobian for KUKA LBR Med 7
    
    Robot = LBR_MED();
    n = size(Robot,1);
    
    % Initialize symbolic transformation matrices
    T = cell(n, 1);
    T_accum = eye(4);
    
    % End-effector position (from direct kinematics)
    for i = 1:n
        T_accum = T_accum * DHTransf(Robot(i,:));
        T{i} = simplify(T_accum);
    end
    
    pe = T{n}(1:3, 4);
    
    % Jacobian columns
    J = sym(zeros(6, n));
    
    % Column 1: Base frame (z0 = [0;0;1], p0 = [0;0;0])
    z0 = [0; 0; 1];
    p0 = [0; 0; 0];
    J(1:3, 1) = cross(z0, pe - p0);
    J(4:6, 1) = z0;
    
    % Columns 2 to n
    for i = 2:n
        zi_1 = T{i-1}(1:3, 3);
        pi_1 = T{i-1}(1:3, 4);
        J(1:3, i) = cross(zi_1, pe - pi_1);
        J(4:6, i) = zi_1;
    end
    
    J = simplify(J);
end
