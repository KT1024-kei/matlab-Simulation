clc; clear; close all;

%% Initial Settings
T = 0.01;                               % Sampling Time
iterations = 900;                      % Iteration Number
r = sqrt(2);                            % Radius of Circular Trajectory
R = 1.0;                                  % Diameter of Agent
c1 = [-1; 1.05; -10];                            % Center of Circular Trajectory of Agent 1
c2 = [1; -1; -10];                          % Center of Circular Trajectory of Agent 2
scale2 = 1.05;                          % Velocity Scale of Agent 2 relative to Agent 1
p10 = c1;                               % Initial Positin of Agent 1
p20 = c2;                               % Initial Positin of Agent 2
pd1 = [];                               % Desired Trajectory of Agent 1
pd2 = [];                               % Desired Trajectory of Agent 2
tp1 = p10;                              % Temporal Position Data of Agent 1
tp2 = p20;                              % Temporal Position Data of Agent 2
k = 0.01;                                 % P Gain
alpha = 0.1;                             % Gain for CBF Condition
p1 = [];                                % Position Data Series of Agent 1
p2 = [];                                % Position Data Series of Agent 2


%% Discrete-time State Update
for t = 1:iterations
    
    
%     plot
    % Desired Velocity Input
    pd1 = c1 - [-10; 10; 0];
    pd2 = c2 - [10; -10; 0];
    vnom1 = k*(pd1 - tp1); % + T*r*[-sin(T*t); cos(T*t)]; 
    vnom2 = k*(pd2 - tp2); % + scale2*T*r*[-sin(scale2*T*t); cos(scale2*T*t)]; 
    
    % QP Settings
    H1 = eye(3)/2;
    f1 = -vnom1;
    A1 = ([0;0;2 * tp2(3)] - [0;0;2 * tp1(3)])';
    diff1 = tp2 - tp1;
    b1 = alpha*((diff1(1)^2 + diff1(2)^2 + diff1(3)^2/100)  - R^2)/4;
    H2 = eye(3)/2;
    f2 = -vnom2;
    A2 = -A1;
    b2 = alpha*((diff1(1)^2 + diff1(2)^2 + diff1(3)^2/100) - R^2)/4;

    % Solve QP
    v1 = quadprog(H1, f1, A1, b1);
    v2 = quadprog(H2, f2, A2, b2);

    
    
    % Position Update
    tp1 = tp1 + T*v1;% + [randn() * 0; randn() * 0; randn()]/1000;
    tp2 = tp2 + T*v2;% + [randn() * 0; randn() * 0; randn()]/1000;
    
    p1 = [p1 tp1];
    p2 = [p2 tp2];
%     figure(1)
%     hold on
% %     plot(p1(1, :), p1(2,:), '.')
% %     plot(p2(1, :), p2(2,:), '.')
%     plot3(tp1(1), tp1(2), tp1(3), '.')
%     plot3(tp2(1), tp2(2), tp2(3),  '.')
%     
%     hold off
end


%% Plot Results
figure
plot3(p1(1,:), p1(2,:), p1(3,:), 'r-', 'LineWidth', 3)
hold on; grid on;
plot3(p2(1,:), p2(2,:), p2(3,:), 'b-', 'LineWidth', 3)
axis equal;
set(gca, 'FontSize', 20);

%% c
figure(1)
h = gca;
view(3)

hold(gca, 'on');
agent1 = plot3(gca, p1(1,1), p1(2,1), p1(3,1), '.');
agent2 = plot3(gca, p2(1,1), p2(2,1), p2(3,1), '.');
xlim([-10 10]);ylim([-10 10]);zlim([-10 10]);
hold(gca, 'off');
for i = 1:iterations
    figure(1)
    
    grid on;
    set(agent1, ...
        'Xdata', p1(1, i), ...
        'Ydata', p1(2, i), ...
        'Zdata', p1(3, i));
    set(agent2, ...
        'Xdata', p2(1, i), ...
        'Ydata', p2(2, i), ...
        'Zdata', p2(3, i));
    axis equal;
    xlim([-1 1]);ylim([-1 1]);zlim([-11 -7]);
    drawnow;
    
end