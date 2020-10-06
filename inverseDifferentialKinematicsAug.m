function q_dot = inverseDifferentialKinematicsAug(q, x_dot, obs, k0)
    J = getAnaliticJacobian(q);
    J_pinv = pinv(J);
    [dists,grads] = distancesAndGrads(q, obs);
    [~,index] = min(dists);
    q0_dot = k0*grads(index,:)';
    q_dot = J_pinv * x_dot + (eye(7)-J_pinv*J)*q0_dot;
end
