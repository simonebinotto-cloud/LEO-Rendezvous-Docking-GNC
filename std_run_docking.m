clearvars; close all; clc;
format long

% Main runner for the project
EarthRad = 6371e3; % Approx
muEarth = 3.986004418e14;

% Initial conditions
%% Orbit parameters
target.h0 = 400e3;
target.mass = 10000;
target.r0abs = EarthRad + target.h0;
target.r0 = [target.r0abs, 0, 0]';
target.v0abs = sqrt(muEarth/target.r0abs);
target.v0 = [0, target.v0abs, 0]';

chaser.h0 = target.h0 - 250;
chaser.mass = 1000;
chaser.r0abs = EarthRad + chaser.h0;
chaser.r0 = [chaser.r0abs; 0; 0]';
chaser.v0 = [0; target.v0abs; 0] + [0.05; 0; 0];
chaser.v0abs = sqrt(chaser.v0(1)^2 + chaser.v0(2)^2 + chaser.v0(3)^2);

%% Docking Port Geometry Configurations
% Target (ISS) Docking Port
% Placed 15 meters below the CoM on the Nadir/R-bar side (-X direction)
target.port_pos_b = [-15; 0; 0]; 
target.port_q_b = [0; 0; 1; 0]; 

% Chaser Docking Port
% Placed 2 meters forward on the Chaser's nose (+X direction)
chaser.port_pos_b = [2; 0; 0];
chaser.port_q_b = [1; 0; 0; 0];

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

%% MPC 
control.MPC = 1;
control.LQR = 0;
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
% Custom Linear Constraints (Approach Cone)
alpha = deg2rad(10); % 10 degree half angle cone
ta = tan(alpha);
k_glide = 0.002;
v_dock = 0.02; % 2 cm/s max contact speed

% Calculate the CoM location where ports touch (the apex of our cone)
% Target port is at -15m, Chaser port sticks out 2m.
% Apex for Chaser CoM is -15 - 2 = -17m.
x_apex = target.port_pos_b(1) - chaser.port_pos_b(1); 

Empc = zeros(5, 3);

% F matrix maps to states [x, y, z, dx, dy, dz]
Fmpc = [ta,  1,  0,  0,  0,  0;   % y + x*tan(alpha) <= G
        ta, -1,  0,  0,  0,  0;   % -y + x*tan(alpha) <= G
        ta,  0,  1,  0,  0,  0;   % z + x*tan(alpha) <= G
        ta,  0, -1,  0,  0,  0;   % -z + x*tan(alpha) <= G
        k_glide, 0, 0, 1, 0, 0];  % Glideslope: k*x + dx <= G
    
% Shift the G matrix so the cone apex and glideslope terminate at x_apex
Gmpc = [x_apex * ta; 
        x_apex * ta; 
        x_apex * ta; 
        x_apex * ta; 
        v_dock + k_glide * x_apex]; 

setconstraint(mpcobj, Empc, Fmpc, Gmpc);

% Calculate dynamic MPC target for port-to-port contact
% Assuming final docked attitude alignment is q = [1; 0; 0; 0]
R_chaser_docked = eye(3); 
chaser_CoM_target = target.port_pos_b - R_chaser_docked * chaser.port_pos_b;

% Update the MPC target state [x, y, z, dx, dy, dz]
xref = [chaser_CoM_target; 0; 0; 0]; 

% Apply Actuator and State Limits
max_thrust_accel = 0.05; % Max specific force (m/s^2)
for i = 1:3
    mpcobj.MV(i).Min = -chaser.mass*max_thrust_accel;
    mpcobj.MV(i).Max =  chaser.mass*max_thrust_accel;
end

% Cost Function Weights
mpcobj.Weights.OV = [1, 500, 500, 10, 50, 50]; 
mpcobj.Weights.MV = [0.01, 1, 1]; 
mpcobj.Weights.MVRate = [0.5, 0.5, 0.5]; 

%% ADCS Controller
ADCS.Kd = 280;
ADCS.Kp = 100;

% Docking Attitude (Aligned to face the ISS port)
q_flip = [0, 0, 1, 0];
q_aligned_ports = quatmultiply(target.port_q_b', q_flip);
q_ref_row = quatmultiply(q_aligned_ports, quatconj(chaser.port_q_b'));
ADCS.q_ref_docking = q_ref_row';

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

%% Simulation Activation Flags
% Forced to MPC for the docking sequence
control.LQR = 0;
control.MPC = 1;

SimTime =  10000;
if control.state == 0
    SimTime = 500;
end

%%
mpc_controller = mpcobj;
res = sim("DockingSimulator.slx");

%% --- 3D DOCKING ANIMATION ---
disp('Starting 3D Docking Animation...');

% 1. Extract Data from 'res' object
time = res.tout;

% Extract position (Handling the struct format shown in your screenshot)
r_rel = squeeze(res.rRelative.signals.values)';

% Extract quaternion (From the new To Workspace block)
q_chaser = res.q.signals.values;  % Chaser quaternion [w, x, y, z] (N x 4)

% Decimate data to speed up animation (plot every 10th frame)
skip = 100; 
time = time(1:skip:end);
r_rel = r_rel(1:skip:end, :);
q_chaser = q_chaser(1:skip:end, :);

% 2. Setup Figure and Axes
figure('Name', 'Docking Animation', 'Color', 'w', 'Position', [100, 100, 800, 600]);
ax = axes('XLim', [-30 10], 'YLim', [-15 15], 'ZLim', [-15 15]);
view(3); grid on; hold on;
xlabel('X (LVLH - R-bar)'); ylabel('Y (LVLH - V-bar)'); zlabel('Z (LVLH - H-bar)');
title('Chaser Approach and Docking (Tracking Camera)');

% 3. Draw Target (ISS) at Origin
[X, Y, Z] = sphere(20);
surf(X*15, Y*15, Z*15, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Mark the Target Docking Port & Connecting Line
plot3(-15, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); 
plot3([0, -15], [0, 0], [0, 0], 'r-', 'LineWidth', 3); 

% 4. Create Chaser Object using hgtransform
chaser_transform = hgtransform('Parent', ax);

% Draw Chaser Body (Attached to transform)
chaser_body = surf(X*2, Y*2, Z*2, 'Parent', chaser_transform, 'FaceColor', 'green', 'EdgeColor', 'none');

% Draw Chaser Docking Port & Connecting Line
chaser_port = plot3(2, 0, 0, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm', 'Parent', chaser_transform);
plot3([0, 2], [0, 0], [0, 0], 'm-', 'LineWidth', 3, 'Parent', chaser_transform);

% Draw Chaser Orientation Axes (+X red, +Y green, +Z blue)
quiver3(0,0,0, 4,0,0, 'r', 'LineWidth', 2, 'Parent', chaser_transform);
quiver3(0,0,0, 0,4,0, 'g', 'LineWidth', 2, 'Parent', chaser_transform);
quiver3(0,0,0, 0,0,4, 'b', 'LineWidth', 2, 'Parent', chaser_transform);

% Draw Trajectory line (Static)
plot3(r_rel(:,1), r_rel(:,2), r_rel(:,3), 'k--', 'LineWidth', 1);

% 5. Animation Loop
zoom_window = 20; % Distance in meters to keep in frame around the chaser

for i = 1:length(time)
    if ~isvalid(chaser_transform)
        disp('Animation closed by user.');
        break; % Exit the loop cleanly if the window is closed
    end
    pos = r_rel(i, :);
    quat = q_chaser(i, :); 
    
    % Convert quaternion to Rotation Matrix
    R = quat2rotm(quat);
    
    % Create 4x4 Transformation Matrix
    T_matrix = eye(4);
    T_matrix(1:3, 1:3) = R;
    T_matrix(1:3, 4) = pos';
    
    % Apply transformation to chaser
    set(chaser_transform, 'Matrix', T_matrix);
    
    % --- Dynamic Tracking Camera ---
    % Automatically shifts the axes to follow the Chaser's exact position
    xlim([pos(1) - zoom_window, 0]);
    ylim([pos(2) - zoom_window, pos(2) + zoom_window]);
    zlim([pos(3) - zoom_window, pos(3) + zoom_window]);
    
    drawnow;
    pause(0.01);
end
disp('Animation Complete.');