%Parameters
l_cm = 0.015; %[m]
m_heli = 0.479; %[kg]
J_eq_p = 0.0172; %[kg*m^2]
J_eq_y = 0.0210; %[kg*m^2]
g = 9.81; %[m*s^2]

K_pp = 0.0556; %[Nm/V]
K_yy = 0.21084; %[Nm/V]
K_py = 0.005; %[Nm/V]
K_yp = 0.15; %[Nm/V]
B_p = 0.01; %[N/V]
B_y = 0.08; %[N/V]

%Operating points/Initial conditions:
theta_op = -10*pi/180;
psi_op = pi/2;
w_theta_op = 0;
w_psi_op = 0;

x_op = [theta_op;psi_op;w_theta_op;w_psi_op];

%control input voltage operating point:
V_mp_op = (K_yy*m_heli*g*cos(theta_op)*l_cm)/(K_yy*K_pp-K_yp*K_py);
V_my_op = (K_yp*V_mp_op)/K_yy;

D_p = J_eq_p+m_heli*l_cm.^2; %Pitch denominator
D_y = J_eq_y+m_heli*cos(theta_op).^2*l_cm.^2; %Yaw denominator

%System matrices
Ac = [0,0,1,0; 
    0,0,0,1;
    (m_heli*g*l_cm*sin(theta_op))/(D_p),0,-B_p/(D_p),0;
    0,0,0,-B_y/(D_y)];

Bc = [0,0;
    0,0;
    K_pp/(D_p),-K_py/(D_p);
    K_yp/(D_y), -K_yy/(D_y)];

 Cc = [1,0,0,0;0,1,0,0];
 D = [0 0; 0 0];



H = zeros(2,4);
G  = eye(4);                    % process noise input
Q = diag([1,1,1,1]);   % modest distrust, more on velocities
R = diag([0.1,0.1]); %measurement noise
plant = ss(Ac,[Bc,G],Cc,0); %D = 0 since it is not present
[~, L, ~, ~] = kalman(plant, Q, R);



 % Discrete time model 
dt = 0.1; %sampling time 
sys = ss(Ac,Bc,Cc,0); %there is no D matrix, so set it as 0 
ds = c2d(sys,dt); 
Ad = ds.a; Bd = ds.b; Cd = ds.c; %discrete time system matrices

