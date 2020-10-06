clear;
clc;
%% Parameters

% Controller gain
Kp_joint = eye(7)*20;
Kp_cart = eye(3)*15;
% Redundancy gain
k0 = 10; %0 for standard (non collision-free) motion planning 
% Integration Time Step
Ts = 0.05;
% Obstacles
obs = [[-0.025;-0.55;0.8]];
% Time execution
T_tot = 20;
% Controller type
controller = "cartesian";

%% Trajectory setup
way_points = [[-0.525;-0.2;0.9],[-0.025;-0.6;0.5],[0.425;-0.3;0.7],[-0.025;-0.55;1.1],[-0.525;-0.2;0.9]];
x_d_des = [[0;0;0],[1;0;0],[0;0;-1],[-1;0;0],[0;0;0]];
% x_d_des = [[0;0;0],[0.9629;-0.0951;-0.2222],[0.0165;0.4465;0.2695],-[0.9629;-0.0951;-0.2222],[0;0;0]];

plot3(way_points(1,:),way_points(2,:),way_points(3,:), 'o','Color','r','MarkerSize',5);
[X,Y,Z] = sphere;
R = 0.15;
X = R*X;
Y = R*Y;
Z = R*Z;
surf(X+obs(1),Y+obs(2),Z+obs(3));
axis equal;

grid on;
hold on;

tvec = 0:Ts:T_tot;
tpts = 0:T_tot/(size(way_points,2)-1):T_tot;

[x,x_dot,x_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', x_d_des);

plot3(x(1,:),x(2,:),x(3,:), 'o','Color','b','MarkerSize',5);
grid on;
hold on;


%% Simulation
robot = VrepConnector(19999,0.05);
q = robot.GetState();
while(norm(directKinematics(q) - way_points(:,1)) > 1e-2 )
    q = robot.GetState();
end

qd = robot.GetState();

for i=1:size(x_dot,2)
    q = robot.GetState();

    if controller == "joint"
        [qd, q_control_dot] = jointSpaceController(qd, x_dot(:,i), q, Ts, Kp_joint, obs(:,1), k0);
    else
        [qd, q_control_dot] = cartesianSpaceController(x(:,i), x_dot(:,i), q, Ts, Kp_cart, obs(:,1), k0);
    end


    x_control = directKinematics(qd);
    plot3(x_control(1),x_control(2),x_control(3), 'o','Color','r','MarkerSize',5);
    hold on;

    x_robot = directKinematics(robot.GetState());
    plot3(x_robot(1),x_robot(2),x_robot(3), 'o','Color','g','MarkerSize',5);
    hold on;

    robot.ApplyControl(q_control_dot, Ts);
end

