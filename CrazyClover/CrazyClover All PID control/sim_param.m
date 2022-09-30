%% physical parametrer
mass = 0.5;
gravity = 9.8;

%% pid gain

% Position
Pos.X.kp = 1.5;
Pos.X.ki = 0;
Pos.X.kd = 0;

Pos.Y.kp = 1.5;
Pos.Y.ki = 0;
Pos.Y.kd = 0;

Pos.Z.kp = 1;
Pos.Z.ki = 0;
Pos.Z.kd = 0;

% Velocity
Vel.X.kp = 5;
Vel.X.ki = 1;
Vel.X.kd = 0;

Vel.Y.kp = 5;
Vel.Y.ki = 1;
Vel.Y.kd = 0;

Vel.Z.kp = 25;
Vel.Z.ki = 10;
Vel.Z.kd = 0;

% Attitude
Roll.kp = 10;
Roll.ki = 5;
Roll.kd = 0;

Pitch.kp = 10;
Pitch.ki = 5;
Pitch.kd = 0;

Yaw.kp = 5;
Yaw.ki = 3;
Yaw.kd = 0;

% Attitude rate
Rollrate.kp = 50;
Rollrate.ki = 50;
Rollrate.kd = 0;

Pitchrate.kp = 50;
Pitchrate.ki = 50;
Pitchrate.kd = 0;

Yawrate.kp = 30;
Yawrate.ki = 3;
Yawrate.kd = 0;

%% trajectory gain
traj.kP = 1.0;
traj.kV = 1.0;
traj.kRx = 100;
traj.kRy = 100;
traj.kRz = 10;
traj.kW = 1.0;
%% integrate limit

Roll.intupper = 10000;
Roll.intlower = -10000;
Pitch.intupper = 10000;
Pitch.intlower = -10000;
Yaw.intupper = 500;
Yaw.intlower = -500;

Rollrate.intupper = 20000;
Rollrate.intlower = -20000;
Pitchrate.intupper = 20000;
Pitchrate.intlower = -20000;
Yawrate.intupper = 20000;
Yawrate.intlower = -20000;

%% controller matrix
kF = 1;
kR = 1.0;
kP = 1.0;
kY = 1.0;
controller_matrix = [mass/(4*gravity)*1000 mass/(4*gravity)*1000 mass/(4*gravity)*1000 mass/(4*gravity)*1000;
                     1/4           1/4          -1/4           -1/4;
                     -1/4           1/4          1/4           -1/4;
                     -1/4           1/4          -1/4          1/4]';
                 
                     

%% physical param
m = 0.5;
Armlength = 0.565/2;

kthrust = 1;
kM1 = Armlength /sqrt(2);
kM2 = Armlength /sqrt(2);
kM3 = 1;
% kM1 = 0;
% kM2 = 0;
% kM3 = 0;
