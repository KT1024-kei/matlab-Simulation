%% 問題設定
A = 1; B = 1; C = 1; % システム
Q = 1;               % システム雑音の分散
R = 2;               % 観測雑音の分散
N = 300;             % データ数

%% 観測データの生成

% 雑音の生成
v = randn(N, 1) * sqrtm(Q);     % システム雑音
w = randn(N, 1) * sqrtm(R);     % 観測雑音

% 状態空間モデルを用いた時系列データの生成
x = zeros(N, 1); y = zeros(N, 1);
y(1) = C'*x(1, :)' + w(1);
for k=2:N
   x(k, :) = A*x(k-1)' + B*v(k-1);
   y(k) = C'*x(k, :) + w(k);
end

%% カルマンフィルタによる状態推定

% 状態推定領域の確保
xhat = zeros(N, 1);

% 初期推定量
P = 0;              % 共分散行列
xhat(1, :) = 0;

% 推定量の時間更新
for k=2:N
   % 推定量，共分散行列，カルマンゲイン
   [xhat(k, :) P G] = kf_6_1(A,B,0,C,Q,R,0,y(k),xhat(k-1, :), P);
end

%% 描画
figure(1), clf
plot(1: N, y, 'k:', 1:N, x, 'r--', 1:N, xhat, 'b-')
xlabel('No. of samples')
legend('mesured', 'true', 'estimate')

