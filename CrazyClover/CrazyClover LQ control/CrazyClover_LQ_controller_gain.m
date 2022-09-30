%% physical parametrer
mass = 0.5;
gravity = 9.8;
g = 9.8;

% 
stateA = [0  0  0  1  0  0  0  0  0  ;
          0  0  0  0  1  0  0  0  0  ;
          0  0  0  0  0  1  0  0  0  ;
          0  0  0  0  0  0  0  g  0  ;
          0  0  0  0  0  0 -g  0  0  ;
          0  0  0  0  0  0  0  0  0  ;
          0  0  0  0  0  0  1  0  0  ;
          0  0  0  0  0  0  0  1  0  ;
          0  0  0  0  0  0  0  0  1  ];
%         X  Y  Z Xd Yd Zd  R  P  Y Wx Wy Wz
I = eye(3) * 10^(-3);
% I = [1 0 0;
%      0 1 0;
%      0 0 1] * 10^(-6);
Jxx = I(1, 1);
Jyy = I(2, 2);
Jzz = I(3, 3);
% 可制御
stateB = [0   0      0     0;
          0   0      0     0;
          0   0      0     0;
          0   0      0     0;
          0   0      0     0;
          1/m 0      0     0;
          0   1      0     0;
          0   0      1     0;
          0   0      0     1];
% 可観測
 stateC = eye(9);
 % 直達項なし
 stateD = zeros(9, 4);
      
% 最適レギュレータ
Q = eye(9);       % 重み行列
Q(4, 4) = 100; Q(5, 5) = 100; Q(6, 6) = 100;
Q(7, 7) = 100; Q(8, 8) = 100; Q(9, 9) = 100;
% Q(10, 10) = 100; Q(11, 11) = 100; Q(12, 12) = 1;
R = eye(4);         
N = zeros(9, 4);
% ゲインを計算
LQRK = lqr(stateA, stateB, Q, R, N);
%% Attitude rate gain
Rollrate.kp = 2;
Rollrate.ki = 5;
Rollrate.kd = 0;

Pitchrate.kp = 2;
Pitchrate.ki = 5;
Pitchrate.kd = 0;

Yawrate.kp = 2;
Yawrate.ki = 5;
Yawrate.kd = 0;

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
                     1           1          -1           -1;
                     -1           1          1           -1;
                     -1           1          -1           1]';
%% physical param
m = 0.5;
Armlength = 0.565/2;

kthrust = 1;
kM1 = Armlength /sqrt(2);
kM2 = Armlength /sqrt(2);
kM3 = 1;