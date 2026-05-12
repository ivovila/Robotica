function [q] = LBR_MED_IK(p_des, R_des, q_prev)
    % LBR_MED_IK  Analytical Inverse Kinematics for KUKA LBR Med 7
    %   p_des: Desired EE position [3x1]
    %   R_des: Desired EE orientation [3x3]
    %   q_prev: Previous joint angles (optional, for choosing solution)
    
    % Robot parameters
    d1 = 0.340;
    d3 = 0.400;
    d5 = 0.400;
    d7 = 0.126;
    
    % 1. Find wrist center pw
    % pw = p_des - d7 * R_des * [0;0;1]
    ae = R_des(:, 3);
    pw = p_des - d7 * ae;
    
    xw = pw(1);
    yw = pw(2);
    zw = pw(3);
    
    % 2. Solve for q1, q2, q4 (fixing q3 = 0)
    % There are two solutions for q1: azimuth and azimuth + pi
    q1_opts = [atan2(yw, xw), atan2(-yw, -xw)];
    
    % Law of cosines for q4 (independent of q1)
    % R^2 = X^2 + Z^2 = d3^2 + d5^2 + 2*d3*d5*cos(q4)
    % where X^2 + Z^2 = xw^2 + yw^2 + (zw - d1)^2
    Z = zw - d1;
    dist_sq = xw^2 + yw^2 + Z^2;
    c4 = (dist_sq - d3^2 - d5^2) / (2 * d3 * d5);
    c4 = max(-1, min(1, c4)); % Clamp for numerical stability
    s4 = sqrt(1 - c4^2);
    q4_opts = [atan2(s4, c4), atan2(-s4, c4)];
    
    % Combine q1 and q4 to get 4 possible solutions for (q1, q2, q4)
    q_sol = zeros(7, 8);
    sol_count = 0;
    
    for i1 = 1:2
        q1 = q1_opts(i1);
        X = xw * cos(q1) + yw * sin(q1);
        
        for i4 = 1:2
            q4 = q4_opts(i4);
            s4_val = sin(q4);
            c4_val = cos(q4);
            
            A_mat = [-(d3 + d5*c4_val), d5*s4_val;
                      d5*s4_val,      (d3 + d5*c4_val)];
            
            if abs(det(A_mat)) < 1e-9
                s2 = 0; c2 = 1; % Singularity
            else
                sol2 = A_mat \ [X; Z];
                s2 = sol2(1);
                c2 = sol2(2);
            end
            q2 = atan2(s2, c2);
            q3 = 0; % Fixed
            
            % 3. Orientation (q5, q6, q7)
            A1 = DH_matrix(d1, q1, 0, pi/2);
            A2 = DH_matrix(0, q2, 0, -pi/2);
            A3 = DH_matrix(d3, q3, 0, -pi/2);
            A4 = DH_matrix(0, q4, 0, pi/2);
            
            R04 = (A1*A2*A3*A4);
            R04 = R04(1:3, 1:3);
            R47 = R04' * R_des;
            
            r33 = R47(3,3);
            c6 = max(-1, min(1, r33));
            s6_val = sqrt(1 - c6^2);
            
            % Solution 1: s6 >= 0
            sol_count = sol_count + 1;
            q6 = atan2(s6_val, c6);
            if abs(s6_val) < 1e-6
                q5 = 0;
                if c6 > 0
                    q7 = atan2(R47(2,1), R47(1,1));
                else
                    q7 = atan2(R47(2,1), -R47(1,1));
                end
            else
                q5 = atan2(-R47(2,3), -R47(1,3));
                q7 = atan2(-R47(3,2), R47(3,1));
            end
            q_sol(:, sol_count) = [q1; q2; q3; q4; q5; q6; q7];
            
            % Solution 2: s6 < 0
            if abs(s6_val) >= 1e-6
                sol_count = sol_count + 1;
                q6_alt = atan2(-s6_val, c6);
                q5_alt = atan2(R47(2,3), R47(1,3));
                q7_alt = atan2(R47(3,2), -R47(3,1));
                q_sol(:, sol_count) = [q1; q2; q3; q4; q5_alt; q6_alt; q7_alt];
            end
        end
    end
    
    % Return one solution (closest to q_prev if provided)
    if nargin > 2 && ~isempty(q_prev)
        % For each joint, consider 2*pi periodicity
        diff_mat = q_sol(:, 1:sol_count) - repmat(q_prev(:), 1, sol_count);
        diff_mat = atan2(sin(diff_mat), cos(diff_mat)); % Map to [-pi, pi]
        dists = sum(diff_mat.^2, 1);
        [~, idx] = min(dists);
        q = q_sol(:, idx);
    else
        q = q_sol(:, 1);
    end
end


function A = DH_matrix(d, theta, a, alpha)
    % Standard DH transformation matrix
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    A = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
         0,   sa,     ca,    d   ;
         0,   0,      0,     1   ];
end
