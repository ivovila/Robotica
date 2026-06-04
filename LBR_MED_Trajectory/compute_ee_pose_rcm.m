function [p_ee, R_ee, a_hat] = compute_ee_pose_rcm(m_target, c_trocar, L_tool)
% Compute end-effector pose enforcing the Remote Centre of Motion (trocar) constraint.
%
%   m_target  3x1  desired tooltip position
%   c_trocar  3x1  fixed trocar position
%   L_tool    scalar  tool length from EE flange to tooltip (m)
%
% The tool axis (EE z-axis) must pass through c_trocar.
% Ordering along the axis:  EE  -->  c_trocar  -->  m_target
% Required: ||m_target - c_trocar|| < L_tool

a      = m_target - c_trocar;    % vector from trocar to target
a_hat  = a / norm(a);            % unit tool-axis direction (= EE z-axis)
p_ee   = m_target - L_tool * a_hat;   % place EE so tip lands on m_target

% Build full rotation matrix (z = a_hat, x/y chosen with a fixed reference)
ref = [0; 1; 0];
if abs(dot(a_hat, ref)) > 0.99
    ref = [1; 0; 0];
end
x_hat = cross(ref, a_hat);
x_hat = x_hat / norm(x_hat);
y_hat = cross(a_hat, x_hat);
R_ee  = [x_hat, y_hat, a_hat];
end
