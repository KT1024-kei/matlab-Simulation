%%
clear; clc; close all

%% 問題設定
A = 1 ; B = 1; C = 1; % システム
Q = 0.4760^2;               % システム雑音の分散
R = 1;               % 観測雑音の分散

%% 観測データの取得
data = readmatrix('..\data\sample_data.xlsx');
t = data(:,1);              % 時間データ
X = data(:, 3);             % 位置データ
X_dot = data(:, 4);         % 差分微分法による速度
X_dot_LPF = data(:, 5);     % 線形フィルタによる速度

N = length(t);              % データ数
y = X_dot;                  % 観測データ
%% カルマンフィルタによる状態推定

% 状態推定領域の確保
xhat = zeros(N, 2);

% 初期推定量
P = 0;              % 共分散行列
xhat(1, :) = 0;

% 推定量の時間更新
for k=2:N
    A = [1 t(k)-t(k-1);0 1];
    B = [1 0]';
    C = [1 0]';
   % 推定量，共分散行列，カルマンゲイン
   [xhat(k, :) P G] = kf_function(A,B,0,C,Q,R,0,y(k),xhat(k-1, :), P);
end

%% First Order Low-Path filter
%{
% url
% https://qiita.com/motorcontrolman/items/39d4abc6c4862817e646

LPF;
              1
  Y(s) = =========== R(s)
          1  + tau*s

tau; 1/(2*pi*F)


離散化

( T + tau(1 - z^(-1)) * y[n] = R * r[n]
-> y[n] = tau / (T + tau) * y[n-1] + T/(T + tau) * r[n]

alpha = tau / (T + tau)とすると

-> y[n] = alpha * y[n-1] + (1 - alpha) * r[n]
%}
F = 10;              % カットオフ周波数
tau = 1/(2*pi*F);     % 時定数
T = 0.01;
alpha = tau/(T + tau);

X_dot_LPF = zeros(length(y)+1, 1);
for i = 1:length(y)
    X_dot_LPF(i+1) = alpha * X_dot_LPF(i, 1) + (1 - alpha) * y(i, 1);
end
% X_dot_LPF = X_dot_LPF(1:end-1);

X_dot_LPF = lowpass(y, 5, 1000);

%% Second Order Low-Path filter using biquad filter with bilinear z transform

% url
% http://www.earlevel.com/main/2003/03/02/the-bilinear-z-transform/
% https://en.wikipedia.org/wiki/Digital_biquad_filter
% https://www.g200kg.com/jp/docs/makingeffects/78743dea3f70c8c2f081b7d5187402ec75e6a6b8.html

F = 10;              % カットオフ周波数
tau = 1/(2*pi*F);    % 時定数
Q = 1/sqrt(2.00);       % quality factor 共進周波数における信号の鋭さ　
T = 0.01;

K = tan(T/(2*tau));
poly = K^2 + K/Q + 1.0;
a(1) = 2.0*(K^2 - 1.0)/poly;   % denominator gains
a(2) = (K^2 - K/Q + 1.0)/poly;
b(1) = K^2/poly;               % numerator gains
b(2) = 2.0 * b(1);

i(1) = 0;   % input history
i(2) = 0;
o(1) = 0;   % output history
o(2) = 0;


X_dot_LPF2P = zeros(length(y)+1, 1);

for n = 1:length(y)
    X_dot_LPF2P(n+1) =  b(1) * y(n) + b(2) * i(1) + b(1) * i(2)...
                       -a(1) * o(1) - a(2) * o(2);
    i(2) = i(1);
    i(1) = y(n);
    o(2) = o(1);
    o(1) = X_dot_LPF2P(n+1);
end
X_dot_LPF2P = X_dot_LPF2P(1:end-1);

% より効率化した計算方法
T = 0.01;
fc = 5;
fr = 1/(fc * T);
omega = tan(pi/fr);

c = 1 + 2*cos(pi/4) * omega + omega^2;

b0 = omega^2/c;
b1 = 2*b0;
b2 = b0;
a1 = 2*(omega^2 - 1)/c;
a2 = (1 - 2*cos(pi/4)*omega + omega^2)/c;

X_dot_LPF_c = zeros(length(y)+1, 1);
r0 = 0;
r1 = 0;
r2 = 0;

for n =1:length(y)
   r0 = y(n) - r1*a1 - r2*a2;
   X_dot_LPF_c(n+1) = r0*b0 + r1*b1 + r2*b2;
   
   r2 = r1;
   r1 = r0;
end

X_dot_LPF2P = X_dot_LPF_c(2:end);
%% 描画
figure(1), clf
plot(t, y, 'k:', t, X_dot_LPF2P, 'b-', t, X_dot_LPF, 'r--', t, xhat(:, 1), 'g--' )
xlabel('No. of samples')
legend('Pderivertive', '2DLPF', '1DLPF', 'kalman')

