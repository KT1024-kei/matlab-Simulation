clc; clear; close all;

%% Initial Settings
T = 0.01;                               % Sampling Time
iterations = 3000;                      % Iteration Number
r = sqrt(2);                            % Radius of Circular Trajectory
R = 0.1;                                  % Diameter of Agent
c1 = [0.5; 0.5];                            % Center of Circular Trajectory of Agent 1
c2 = [-1; -1];                          % Center of Circular Trajectory of Agent 2
scale2 = 1.05;                          % Velocity Scale of Agent 2 relative to Agent 1
p10 = c1 + r*[cos(0); sin(0)];          % Initial Positin of Agent 1
p20 = c2 + r*[cos(0+pi); sin(0+pi)];    % Initial Positin of Agent 2
pd1 = [];                               % Desired Trajectory of Agent 1
pd2 = [];                               % Desired Trajectory of Agent 2
tp1 = p10;                              % Temporal Position Data of Agent 1
tp2 = p20;                              % Temporal Position Data of Agent 2
k = 10;                                 % P Gain
alpha = 100;                             % Gain for CBF Condition
p1 = [];                                % Position Data Series of Agent 1
p2 = [];                                % Position Data Series of Agent 2


%% Discrete-time State Update
for t = 1:iterations
    
    
%     plot
    % Desired Velocity Input
    pd1 = c1 + r*[cos(T*t); sin(T*t)];
    pd2 = c2 + r*[cos(scale2*T*t + pi); sin(scale2*T*t + pi)];
    vnom1 = k*(pd1 - tp1); % + T*r*[-sin(T*t); cos(T*t)]; 
    vnom2 = k*(pd2 - tp2); % + scale2*T*r*[-sin(scale2*T*t); cos(scale2*T*t)]; 
    
    % QP Settings
    H1 = eye(2)/2;
    f1 = -vnom1;
    A1 = (tp2 - tp1)';
    b1 = alpha*(norm(tp2 - tp1)^2 - R^2)/4;
    H2 = eye(2)/2;
    f2 = -vnom2;
    A2 = (tp1 - tp2)';
    b2 = alpha*(norm(tp1 - tp2)^2 - R^2)/4;
    
    % Solve QP
    v1 = quadprog(H1, f1, A1, b1);
    v2 = quadprog(H2, f2, A2, b2);
    
    % Position Update
    tp1 = tp1 + T*v1;
    tp2 = tp2 + T*v2;
    
    p1 = [p1 tp1];
    p2 = [p2 tp2];
    
    figure(1)
    hold on
    plot(p1(1, :), p1(2,:), '.')
    plot(p2(1, :), p2(2,:), '.')
    hold off
end


%% Plot Results
figure
plot(p1(1,:), p1(2,:), 'r-', 'LineWidth', 3)
hold on; grid on;
plot(p2(1,:), p2(2,:), 'b-', 'LineWidth', 3)
axis equal;
set(gca, 'FontSize', 20);
