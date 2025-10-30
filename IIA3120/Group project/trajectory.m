% Setting up waypoints
p1 = [0.5; 0.5; 0.5]; p2 = [0.5; -0.2; 0.1]; p3 = [-0.3; 0; 0.2]; p4 = [0.5; 0.5; 0.5];
wp = [p1, p2, p3, p4];

% Setting up waypoint velocities
v1 = [0; 0; 0]; v2 = [0; 0.7; 0.4]; v3 = [0.32; 0.75; 0.981]; v4 = [0; 0; 0];
wv = [v1, v2, v3, v4];

% Setting up waypoint times
wt = [0; 1; 2; 3];

% Creating a time vector with 0.01 timestep
t = wt(1):0.01:wt(end)+1;

% Create trajectory
traj = cubicpolytraj(wp,  wt, t, VelocityBoundaryCondition=wv);

robot = importrobot("rx150.urdf");

%% Plot
%figure,
%ax = show(robot, "Frames", "off");
%hold on
%plot3(ax, traj(1,:), traj(2,:), traj(3,:), "b.-")
%hold on
%scatter3(ax, wp(1,:), wp(2,:), wp(3,:), "ro", "Linewidth", 2);