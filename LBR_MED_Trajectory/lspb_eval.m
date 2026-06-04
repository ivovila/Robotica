function [s, s_dot] = lspb_eval(tau, T, s_dot_c, t_c, s_ddot_c)
% LSPB_EVAL  Normalized LSPB path profile.  s(0)=0, s(T)=1, ds/dt(0)=ds/dt(T)=0.
%
%   tau      local time in [0, T]
%   T        segment duration
%   s_dot_c  cruise velocity  (must be > 1/T)
%   t_c      blend time      = T - 1/s_dot_c
%   s_ddot_c blend accel     = s_dot_c^2 / (s_dot_c*T - 1)

    tau = max(0, min(tau, T));

    if tau <= t_c
        % Acceleration phase
        s     = 0.5 * s_ddot_c * tau^2;
        s_dot = s_ddot_c * tau;

    elseif tau <= T - t_c
        % Cruise phase
        s     = 0.5 * s_ddot_c * t_c^2 + s_dot_c * (tau - t_c);
        s_dot = s_dot_c;

    else
        % Deceleration phase (symmetric to acceleration)
        tau_r = T - tau;
        s     = 1 - 0.5 * s_ddot_c * tau_r^2;
        s_dot = s_ddot_c * tau_r;
    end
end
