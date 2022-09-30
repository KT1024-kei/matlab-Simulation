%% 問題設定
A = 1 ; B = 1; C = 1; % システム
Q = 0.4760^2;               % システム雑音の分散
R = 1;               % 観測雑音の分散

%% 観測データの取得
data = readmatrix('..\data\sample_data.xlsx');
T = data(:,1);              % 時間データ
X = data(:, 3);             % 位置データ
X_dot = data(:, 4);         % 差分微分法による速度
X_dot_LPF = data(:, 5);     % 線形フィルタによる速度

N = length(T);              % データ数
y = X_dot;                  % 観測データ
%% カルマンフィルタによる状態推定

% 状態推定領域の確保
xhat = zeros(N, 1);

% 初期推定量
P = 0;              % 共分散行列
xhat(1, :) = 0;

% 推定量の時間更新
for k=2:N
   % 推定量，共分散行列，カルマンゲイン
   [xhat(k, :) P G] = kf_function(A,B,0,C,Q,R,0,y(k),xhat(k-1, :), P);
end

%% 描画
figure(1), clf
plot(1: N, y, 'k:', 1:N, X_dot_LPF, 'r--', 1:N, xhat, 'b-')
xlabel('No. of samples')
legend('Pderivertive', 'LPF', 'estimate')

