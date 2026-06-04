function R = rodrigues_rot(r, theta)
% RODRIGUES_ROT  Rotation matrix from unit axis r and angle theta.
%   R = I + sin(theta)*K + (1-cos(theta))*K^2,  K = skew(r)

    if abs(theta) < 1e-12
        R = eye(3);
        return;
    end

    K = [   0,  -r(3),  r(2);
         r(3),     0, -r(1);
        -r(2),  r(1),    0 ];

    R = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
end
