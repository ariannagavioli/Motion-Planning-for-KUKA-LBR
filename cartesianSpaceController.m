function [qd, q_control_dot] = cartesianSpaceController(x, x_dot, q, Ts, Kp, obs, k0)
% Kinematic control of cartesian motion
    x_robot = directKinematics(q);
    ep = x - x_robot;
    control_signal = Kp*ep;
    control_signal = control_signal + x_dot;
    q_control_dot = inverseDifferentialKinematicsAug(q, control_signal, obs, k0);  
    qd = q + q_control_dot*Ts;
end

