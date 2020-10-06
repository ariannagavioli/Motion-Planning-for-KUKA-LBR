function [qd_new, q_control_dot] = jointSpaceController(qd, xd, q, Ts, Kp, obs, k0)
% Kinematic control of joint motion
    qd_dot = inverseDifferentialKinematicsAug(qd, xd, obs, k0);
    qd_new = qd + qd_dot*Ts;          %Euler integration
    q_err = qd_new - q;
    control_signal = Kp*q_err;
    q_control_dot = control_signal + qd_dot;
end


