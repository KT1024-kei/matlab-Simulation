function [xhat_new P_new, G] = kf_6_1(A, B, Bu, C, Q, R, u, y, xhat, P)
% 線形カルマンフィルタの更新を行う
% 引数:
%      A, B, h, C 対象システム
%                   x(k+1) = Ax(k) + Bv(k) + Bu u(k)
%                     y(k) = C'x(k) + w(k)
% 
%      Q, R: 雑音v, wの共分散行列. v, w は正規性白色雑音で
%                   E[v(k)] = E[w(k)] = 0
%                   E[v(k)'v(k)] = Q 
%                   E[w(k)'w(k)] = R であることを想定
% 
%      u: 状態更新前時点での制御入力　u(k-1)
%      y: 状態項新辞典での観測出力    y(k)
%      x_hat: 更新前の状態推定値      xhat(k-1)
%      P: 誤差共分散行列              P(k-1)
%      x_hat_new:　更新後の状態推定値 xhat(k)
%      P_new: 更新後の誤差共分散行列  P(k)
%      G: カルマンゲイン              G
% 
% 列ベクトルに成形
xhat = xhat(:); u = u(:); y = y(:);

% 事前推定値
xhatm = A*xhat + Bu*u;
Pm = A*P*A' + B*Q*B';

% カルマンゲイン行列
G = Pm*C/(C'*Pm*C+R);

% 事後推定値
xhat_new = xhatm + G*(y - C'*xhatm);
P_new = (eye(size(A)) - G*C')*Pm;
end