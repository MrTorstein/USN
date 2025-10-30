%% Create trajectory
clc
close all
clear all
format shortG

rx150 = importrobot('rx150.urdf');
figure, ax = show(rx150, 'Frames', 'off');

% smimport('rx150.urdf');

% Trajectory
% Set waypoints in cm
wp1 = [25; -3; 25];
wp2 = [9; -34; 5];
wp3 = [-12; -31; 5];
wp4 = wp3;
wp5 = wp1;

% Convert waypoints to meter
wp = [wp1, wp2, wp3, wp4, wp5]./100;

% Set velosities
vel1 = [0; 0; 0];
vel2 = [-0.1; 0.01; 0.01];
vel3 = [0; 0; 0];
vel4 = [0; 0; 0];
vel5 = [0.3; 0.7; 0];

vel = [vel1, vel2, vel3, vel4, vel5];

% Set time for each waypoint
wpTime = [0; 3; 5; 8; 10];

trajTimes = 0:0.1:wpTime(end);

trajectory = cubicpolytraj(wp, wpTime, trajTimes, 'VelocityBoundaryCondition', vel);

filterConstant = 0.3;


%% Show robot
clc
close all
% Get the joint names from the robot model
jointNames = {rx150.homeConfiguration.JointName};

% Define joint positions
gripper = 20; % Percent open
q = [-8, -38, 38, 0, 0, 0].*pi/180; % Joint positions
q = [q, gripper*0.00027+0.0111, -gripper*0.00027-0.0111];


% Initialize the configuration structure
config = repmat(struct('JointName', '', 'JointPosition', 0), 1, numel(jointNames));

% Populate the configuration structure
for i = 1:numel(q)
    config(i).JointName = jointNames{i};
    config(i).JointPosition = q(i);
end

% Visualize the robot
figure, 
ax = rx150.show(config, 'Frames', 'off');
hold on

% Show trajectory 
scatter3(wp(1,:), wp(2,:), wp(3,:), 'bo', 'LineWidth',2);
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'r.-');
set(gcf,'position',[1500,80, 1500, 1250])

%% Forward Kinematics
clc
format shortG
syms d th a al q1 q2 q3 q4 q5

D = [103.91 0 0 0 65+66+27.575];
Theta = [q1 (q2+pi/2-atan(50/150)) (q3-pi/2+atan(50/150)) q4+pi/2 q5];
A = [0 157.7 150 0 0];
Alpha = [pi/2 0 0 pi/2 0];

A_mat = [cos(th) -sin(th)*cos(al) sin(th)*sin(al) a*cos(th);
     sin(th) cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
     0 sin(al) cos(al) d;
     0 0 0 1];

A_1 = subs(A_mat, [d th a al], [D(1) Theta(1) A(1) Alpha(1)]);
A_2 = subs(A_mat, [d th a al], [D(2) Theta(2) A(2) Alpha(2)]);
A_3 = subs(A_mat, [d th a al], [D(3) Theta(3) A(3) Alpha(3)]);
A_4 = subs(A_mat, [d th a al], [D(4) Theta(4) A(4) Alpha(4)]);
A_5 = subs(A_mat, [d th a al], [D(5) Theta(5) A(5) Alpha(5)]);
A_ee = A_1*A_2*A_3*A_4*A_5;
simplify(A_ee)

Ae0 = double(subs(A_ee, [q1 q2 q3 q4 q5], [0.1 -pi/2 pi/2 0 1]))
Ae0(:,4)