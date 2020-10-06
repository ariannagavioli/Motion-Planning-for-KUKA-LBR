function qd = inverseDifferentialKinematics(q, xd)
    J = getAnaliticJacobian(q);
    J_pinv = pinv(J);
    qd = J_pinv * xd;
end

