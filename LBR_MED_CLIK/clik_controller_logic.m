function q_dot = clik_controller_logic(p_des, R_des, p_curr, R_curr, J, q_curr, q_rest, K_pos, K_ori, Kn_val, p_dot_des, omega_des)
% CLIK_CONTROLLER_LOGIC  Closed-Loop IK with optional feedforward.
%
%   Task-space command:
%     v_e = [ p_dot_des + K_pos*(p_des - p_curr) ]
%           [ omega_des + K_ori * e_ori           ]
%
%   Omitting p_dot_des / omega_des reduces to pure feedback (defaults: zero).

    if nargin < 11 || isempty(p_dot_des), p_dot_des = zeros(3,1); end
    if nargin < 12 || isempty(omega_des),  omega_des  = zeros(3,1); end

    Kp = K_pos * eye(3);
    Ko = K_ori  * eye(3);
    Kn = Kn_val * eye(numel(q_curr));

    ep = p_des(:) - p_curr(:);

    ne = R_curr(:,1);  se = R_curr(:,2);  ae = R_curr(:,3);
    nd = R_des(:,1);   sd = R_des(:,2);   ad = R_des(:,3);
    eo = 0.5 * (cross(ne,nd) + cross(se,sd) + cross(ae,ad));

    v_e = [p_dot_des(:) + Kp * ep;
           omega_des(:) + Ko * eo];

    lambda = 1e-3;
    Jinv   = J' / (J*J' + lambda^2 * eye(6));

    q_dot_p = Jinv * v_e;
    P       = eye(size(J,2)) - Jinv * J;
    q_dot_n = P * Kn * (q_rest(:) - q_curr(:));

    q_dot = q_dot_p + q_dot_n;
end
