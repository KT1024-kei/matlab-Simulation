%% design path drom points by spline

% 軌道の通る点を19点決める
point = [0, 0 
    1, 0
    2, 0
    3, 0.5
    4, 1.5
    4.8, 1.5
    5, 0.8
    6, 0.5
    6.5, 0
    7.5, 0.5
    7, 2
    6, 3
    5, 4
    4., 2.5
    3, 3
    2., 3.5
    1.3, 2.2
    0.5, 2.
    0, 3];

s = 1:1:length(point);

% なめらかな軌道にする
px_spline = spline(s, point(:, 1), 1:0.01:length(point));
py_spline = spline(s, point(:, 2), 1:0.01:length(point));

%% insert yaw
% 軌道の各点の角度を取得？
yaw = zeros(length(px_spline), 1);
for i = 2:length(px_spline)-1
    x_forward = px_spline(i+1);
    x_backward = px_spline(i-1);
    y_forward = py_spline(i+1);
    y_backword = py_spline(i-1);
    
    yaw(i) = atan2(y_forward - y_backword, x_forward - x_backward);
    
end

yaw(1) = yaw(2);
yaw(end) = yaw(end-1);

%% plot with attitude

arrow_scale = 0.01;

figure(101);
% 軌道設計時に決めた各ポイントを描画
plot(point(:, 1), point(:, 2), 'bo-'); hold on;
% 各ポイントからなめらかにした軌道を描画，各ポイントの速度ベクトルも描画する，
% ただしスケールはすべて同じ
quiver(px_spline', py_spline', cos(yaw)*arrow_scale, sin(yaw)*arrow_scale);
plot(px_spline, py_spline, 'r-'); grid on; hold off;

%% save path
path = [px_spline', py_spline', yaw];

save('path', 'path')
