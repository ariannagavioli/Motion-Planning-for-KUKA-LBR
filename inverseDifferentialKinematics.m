function qd = inverseDifferentialKinematics(q, xd)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    J = getAnaliticJacobian(q);
    J_pinv = pinv(J);
    qd = J_pinv * xd;
end

