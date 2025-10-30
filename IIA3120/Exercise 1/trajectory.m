% Setting up waypoints
p1 = [-0.5; -0.5; -0.5]; p2 = [0.15; 0.7; 0.34]; p3 = [0.5; 0.7; 0.5];
wp = [p1, p2, p3];

% Setting up waypoint velocities
v1 = [0; 0; 0]; v2 = [1; 1; 1]; v3 = [1; 1; 1];
wv = [v1, v2, v3];

% Setting up waypoint times
wt = [0; 2; 4];

% Creating a time vector with 0.01 timestep
t = wt(1):0.01:wt(end)+1;

% Create trajectory
traj = cubicpolytraj(wp,  wt, t, VelocityBoundaryCondition=wv);

robot = importrobot("my_universalUR10.urdf");

filterconstant = 0.001;