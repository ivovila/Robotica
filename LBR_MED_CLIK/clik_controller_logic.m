function q_dot = clik_controller_logic(p_des, R_des, p_curr, R_curr, J, q_curr, q_rest, K_pos, K_ori, Kn_val)
    Kp = K_pos * eye(3);
    Ko = K_ori * eye(3);
    K  = [Kp, zeros(3,3); zeros(3,3), Ko];
    
    Kn = Kn_val * eye(size(q_curr, 1));

    ep = p_des(:) - p_curr(:);

    ne = R_curr(:,1); se = R_curr(:,2); ae = R_curr(:,3);
    nd = R_des(:,1);  sd = R_des(:,2);  ad = R_des(:,3);
    eo = 0.5 * (cross(ne, nd) + cross(se, sd) + cross(ae, ad));

    e = [ep; eo];

    % Damped Least Squares (DLS) for robustness near singularities
    k = 1e-3;
    Jinv = J' / (J * J' + k^2 * eye(6));

    v = K * e;
    q_dot_p = Jinv * v;

    P = eye(size(J, 2)) - Jinv * J;
    q_dot_n = P * Kn * (q_rest(:) - q_curr(:));

    q_dot = q_dot_p + q_dot_n;
end
