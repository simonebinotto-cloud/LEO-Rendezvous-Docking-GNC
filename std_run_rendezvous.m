clearvars; close all; clc;
format long

% Main runner for the project

EarthRad = 6371e3; % Approx
muEarth = 3.986004418e14;

% Initial conditions
% Structs for Spacecraft Chaser & Target (Initial Circular Orbits)

%% Orbit parameters
chaser.h0 = 395e3;
chaser.mass = 1000;
chaser.r0abs = EarthRad + chaser.h0;
% Chaser starts behind target by 5 km along-track
delta_theta = -5000/chaser.r0abs;
chaser.r0 = chaser.r0abs * [cos(delta_theta); sin(delta_theta); 0];
chaser.v0abs = sqrt(muEarth/chaser.r0abs);
chaser.v0 = chaser.v0abs * [-sin(delta_theta); cos(delta_theta); 0];
% chaser.r0 = [chaser.r0abs, 0, 0]';
% chaser.v0 = [0, chaser.v0abs, 0]';

target.h0 = 400e3;
target.mass = 10000;
target.r0abs = EarthRad + target.h0;
target.r0 = [target.r0abs, 0, 0]';
target.v0abs = sqrt(muEarth/target.r0abs);
target.v0 = [0, target.v0abs, 0]';

%% Rotation parameters
% Chaser

chaser.q0 = [1 0 0 0]';
chaser.J = 400*eye(3);
chaser.w0 = [0 0 0];

%% MEKF

bias = [5.0e-4; -3.0e-4; 8.0e-4];
dt = 0.01;
q_ic = [1;0;0;0];
P_in = eye(6) * 1e-4;
beta_in = zeros(3,1);

%% Controller
control.state = 1;

% Space State from CW equations

n = sqrt(muEarth/target.r0abs^3);

A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     3*n^2 0 0 0 2*n 0;
     0 0 0 -2*n 0 0;
     0 0 -n^2 0 0 0];
B = [zeros(3,3); eye(3)/chaser.mass];

xref = [-250, 0, 0, 0.05, 0, 0]';
u_ref = -B\(A*xref);

% LQR
% Q = diag([1e-5, 1e-5, 1e-5, 1, 1, 1]); 
% R = diag([1, 1, 1]);
Q = diag([1, 1, 1, 100, 100, 100]);   % position cheap, velocity expensive
R = 1e5 * eye(3);                      

K = lqr(A,B,Q, R);

%% MPC 

C = eye(6); 
D = zeros(6,3);

Ts = 1.0; % Controller sample time 1 Hz
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, Ts, 'zoh');

% Initialize MPC 
PredictionHorizon = 30; % Look 30 steps ahead
ControlHorizon = 5;     % Optimize the next 5 thrust maneuvers
mpcobj = mpc(sys_d, Ts, PredictionHorizon, ControlHorizon);

% Custom Linear Constraints (Approach Cone)
% sqrt(y^2 + z^2) <= -x * tan(alpha)
% Linearized into a 4 sided pyramid  E*u + F*y <= G

alpha = deg2rad(10); % 10 degree half angle cone
ta = tan(alpha);
k_glide = 0.002;
Empc = zeros(5, 3);

% F matrix maps to states [x, y, z, dx, dy, dz]
Fmpc = [ta,  1,  0,  0,  0,  0;   % y + x*tan(alpha) <= 0
    ta, -1,  0,  0,  0,  0;   % -y + x*tan(alpha) <= 0
    ta,  0,  1,  0,  0,  0;   % z + x*tan(alpha) <= 0
    ta,  0, -1,  0,  0,  0;  % -z + x*tan(alpha) <= 0
    k_glide, 0, 0, 1, 0, 0]; % Glideslope: k*x + dx <= 0.02
    


Gmpc = [0; 0; 0; 0; 0.02]; 

setconstraint(mpcobj, Empc, Fmpc, Gmpc);
xref2 = [0, 0, 0, 0, 0, 0]';
% Apply Actuator and State Limits
max_thrust_accel = 0.05; % Max specific force (m/s^2)

for i = 1:3
    mpcobj.MV(i).Min = -max_thrust_accel;
    mpcobj.MV(i).Max =  max_thrust_accel;
end

% Constrain docking approach speed
% Keep moving up the R-bar max speed 0.5 m/s
% mpcobj.OV(4).Min = 0.0;  
% mpcobj.OV(4).Max = 0.5;

% Cost Function Weights
% Penalize Y and Z drift. Less on X so it approaches smoothly.
mpcobj.Weights.OV = [1, 500, 500, 10, 50, 50]; 

% Manipulated Variables [ux, uy, uz]
% Less penalty on ux so the controller fight gravity
mpcobj.Weights.MV = [0.01, 1, 1]; 

% MV Rate (Slew rate)
% Penalize rapid thruster chattering
mpcobj.Weights.MVRate = [0.5, 0.5, 0.5]; 



%% ADCS Controller

ADCS.Kd = 280;
ADCS.Kp = 100;

qrefr = randn(4,1);
qrefr = qrefr/norm(qrefr);

%% Actuators

RCS.l = 1.75; % m
RCS.max = 5; % N

RCS.rA = [RCS.l, RCS.l, RCS.l]';
RCS.rB = [-RCS.l, RCS.l, -RCS.l]';
RCS.rC = [-RCS.l, -RCS.l, RCS.l]';
RCS.rD = [RCS.l, -RCS.l, -RCS.l]';

RCS.dirA = [-1 0 0; 0 -1 0; 0 0 -1];
RCS.dirB = [1 0 0; 0 -1 0; 0 0 1];
RCS.dirC = [1 0 0; 0 1 0; 0 0 -1];
RCS.dirD = [-1 0 0; 0 1 0; 0 0 1];

RCS.B = RCS.max*[cross(RCS.rA, RCS.dirA(:,1)), cross(RCS.rA, RCS.dirA(:,2)), cross(RCS.rA, RCS.dirA(:,3)), ...
             cross(RCS.rB, RCS.dirB(:,1)), cross(RCS.rB, RCS.dirB(:,2)), cross(RCS.rB, RCS.dirB(:,3)), ...
             cross(RCS.rC, RCS.dirC(:,1)), cross(RCS.rC, RCS.dirC(:,2)), cross(RCS.rC, RCS.dirC(:,3)), ...
             cross(RCS.rD, RCS.dirD(:,1)), cross(RCS.rD, RCS.dirD(:,2)), cross(RCS.rD, RCS.dirD(:,3))];

RCS.F = RCS.max*[RCS.dirA, RCS.dirB, RCS.dirC, RCS.dirD];

RCS.Tcycle = 0.5;
RCS.MIB = 0.05;

% % Test rotations
% u = zeros(12,6);
% u([9 6 2 11], 1) = 1;   % +roll
% u([3 12 8 5], 2) = 1;   % -roll
% u([7 10 3 6], 3) = 1;   % +pitch
% u([1 4 9 12], 4) = 1;   % -pitch
% u([1 7 5 11], 5) = 1;   % +yaw
% u([4 10 2 8], 6) = 1;   % -yaw
% disp(RCS.B*u)

%% Simulation

% Type to sim
control.LQR = 1;
control.MPC = 0;

SimTime =  10000;
if control.state == 0
    SimTime = 500;
end

%%

res = sim("RendezvousSimulator.slx");

%% Plots
% plotter
