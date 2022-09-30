%% differential flatness

%% ログ取得
X_ref = out.Position.signals(1).values(1:end,1);
Y_ref = out.Position.signals(1).values(1:end,2);
Z_ref = out.Position.signals(1).values(1:end,3);
X = out.Position.signals(2).values(1:end,1);
Y = out.Position.signals(2).values(1:end,2);
Z = out.Position.signals(2).values(1:end,3);

figure(1)
view(3)
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);
plot3(X_ref, Y_ref, Z_ref, 'LineWidth', 3)
grid on;

%% 姿勢の推移をシミュレーションで見てみる
% 図１の準備
figure(1)
R = out.Attitude.signals.values(:,:,1);
xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([-1.5 1.5]);    % 図の各軸の範囲を設定
p1 = [0.1; 0.1; 0];                                  % ドローン座標系から見た，ドローンのプロペラ位置1
p2 = [-0.1; 0.1;0];                                  %         """           ドローンのプロペラ位置2
p3 = [0.1; -0.1;0];                                  %         """           ドローンのプロペラ位置3
p4 = [-0.1; -0.1; 0];                                %         """           ドローンのプロペラ位置4
pb1 = R(1) * p1;     % ワールド座標系から見た，ドローンのプロペラ位置1
pb2 = R(1) * p2;     %         """           ドローンのプロペラ位置2
pb3 = R(1) * p3;     %         """           ドローンのプロペラ位置3
pb4 = R(1) * p4;     %         """           ドローンのプロペラ位置4
pb = [pb1';pb2';pb3';pb4'];    % ベクトル化
% 三次元プロット
hold(gca, 'on');
figure(1)
view(3)
drone = plot3(gca, pb(:, 1), pb(:, 2), pb(:, 3), 'o', 'MarkerSize', 10 , 'Color', 'r');  % ドローンの位置
fig1_ARM14 = plot3(gca, [pb(1, 1) pb(4, 1)] ,[pb(1, 2) pb(4, 2)], [pb(1, 3) pb(4, 3)], '-bo', 'MarkerSize', 7); % ドローンのプロペラ１と４をつなぐ線
fig1_ARM23 = plot3(gca, [pb(2, 1) pb(3, 1)] ,[pb(2, 2) pb(3, 2)], [pb(2, 3) pb(3, 3)], '-go', 'MarkerSize', 7); % ドローンのプロペラ２と３をつなぐ線
xlabel('X[m]'); % 各軸のラベル
ylabel('Y[m]');
zlabel('Z[m]');
set(gca, 'FontSize', 20);
hold(gca, 'off');
for i = 1:length(X)
    x = [X(i) Y(i) Z(i)]';
    xlim([-2 2]); ylim([-2 2]); zlim([-2 2]);    % 図の各軸の範囲を設定
    figure(1)
    R = out.Attitude.signals.values(:,:,i);
    % ワールド座標から見たドローンのプロペラの位置を計算
    pb1 = x + R * p1;
    pb2 = x + R * p2;
    pb3 = x + R * p3;
    pb4 = x + R * p4;
    pb = [pb1';pb2';pb3';pb4'];
    hold on
    set(drone,'XData', pb(:, 1), 'YData', pb(:, 2),'ZData', pb(:, 3));                                          % ドローンの中心位置を更新
    set(fig1_ARM14,'XData', [pb(1, 1) pb(4, 1)], 'YData', [pb(1, 2) pb(4, 2)],'ZData', [pb(1, 3) pb(4, 3)]);    % ドローンのプロペラ１４をつなぐ棒の位置ドローンの中心位置を更新
    set(fig1_ARM23,'XData', [pb(2, 1) pb(3, 1)], 'YData', [pb(2, 2) pb(3, 2)],'ZData', [pb(2, 3) pb(3, 3)]);    % ドローンのプロペラ２３をつなぐ棒の位置を更新
    hold off
end 
