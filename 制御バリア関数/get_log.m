%% ログ取得
clc;
P_ref = out.Position{2}.Values.Data;
P = out.Position{1}.Values.Data;
R =  out.Attitude{1}.Values.Data;
R_ref =  out.Attitude{2}.Values.Data;
RPY = out.RPY{1}.Values.Data;
RPY_ref = out.RPY{2}.Values.Data;
t = out.tout;

%% 位置プロット
figure(1)
a = plot(gca ,t, [P(1:end, 1) P(1:end,3) P_ref(1:end,1) P_ref(1:end,3)]);

%%
grid on
xlabel('T[ms]');ylabel('Position[m]')
xlim([0, 1]);ylim([-0.2, 0.2]);
set(a, 'LineWidth', 3);
set(gca, 'Fontsize', 20, 'FontName', 'Times')
legend({'$x_{ref}$', '$z_{ref}$', '$x$','$z$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
%% 勢プロット R回転行列
figure(2)
b = plot(t, [R_ref(1:end, 2) R(1:end, 2)]);

%%
grid on
xlabel('T[ms]');ylabel('Attitude')
xlim([0, 5]);ylim([-1.1, 1.1]);
set(b, 'LineWidth', 4);
set(gca, 'FontSize', 20);

%% 姿勢プロット　オイラー角
figure(3)
c = plot(t, [RPY_ref(1:end, 2) RPY(1:end, 2)]);
%%
grid on
xlabel('T[ms]');ylabel('Pitch[rad]')
xlim([0, 4]);ylim([-1.1, 1.1]);
set(c, 'LineWidth', 3);
set(gca, 'FontSize', 20);
legend({'$\theta_{ref}$', '$\theta_b$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');