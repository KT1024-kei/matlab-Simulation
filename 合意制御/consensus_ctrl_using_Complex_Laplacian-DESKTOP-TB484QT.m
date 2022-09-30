%% Distributed Formation Control of Multi-Agent Systems Using Complex Laplacian
clc; clear;
%% 各エージェントの初期位置とつながりを表すグラフ
zeta = [-2 + 2j; 2 + 2j; -1 + 3j; 4j; 1 + 3j; -2; -1 + 1j; 2j; 1 + 1j; 2; -1 - 1j; -2j; 1- 1j; -2 - 2j; -1 - 3j; -4j; 1 - 3j; 2 - 2j];
formation_conf = [ -1  0  3  0  0  6  0  0  0  0  0  0  0  0  0  0  0  0;
                    0 -1  0  0  5  0  0  0  0 10  0  0  0  0  0  0  0  0;
                    1  0 -1  4  0  0  0  0  0  0  0  0  0  0  0  0  0  0;
                    0  0  3 -1  5  0  0  0  0  0  0  0  0  0  0  0  0  0;
                    0  2  0  4 -1  0  0  0  0  0  0  0  0  0  0  0  0  0;
                    1  0  0  0  0 -1  7  0  0  0 11  0  0 14  0  0  0  0;
                    0  0  0  0  0  6 -1  8  0  0  0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  7 -1  9  0  0  0  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  8 -1 10  0  0  0  0  0  0  0  0;
                    0  2  0  0  0  0  0  0  9 -1  0  0 13  0  0  0  0 18;
                    0  0  0  0  0  6  0  0  0  0 -1 12  0  0  0  0  0  0;
                    0  0  0  0  0  0  0  0  0  0 11 -1 13  0  0  0  0  0;
                    0  0  0  0  0  0  0  0  0 10  0 12 -1  0  0  0  0  0;
                    0  0  0  0  0  6  0  0  0  0  0  0  0 -1 15  0  0  0;
                    0  0  0  0  0  0  0  0  0  0  0  0  0 14 -1 16  0  0;
                    0  0  0  0  0  0  0  0  0  0  0  0  0  0 15 -1 17  0;
                    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0 16 -1 18;
                    0  0  0  0  0  0  0  0  0 10  0  0  0  0  0  0 17 -1;
                  ];
%% グラフラプラシアンの構築
k = 1;
p = 1;
omega = zeros(length(zeta));
L = zeros(length(zeta));

% 達成したいフォーメーションのグラフラプラシアンの構築
for n = 1:length(zeta)
    
    % 近傍エージェントが2つより多いい場合
    if length(find(formation_conf(n, :) == 0)) < length(zeta) - 3
        
        neighboors = find(formation_conf(n, :) > 0); % 自分の近傍のエージェントの番号の配列を取得
        two_neighboors = nchoosek(neighboors, 2); % 近傍の二項係数を取得
%         disp(two_neighboors);
        for m = 1:length(two_neighboors(:, 1))
            agents = two_neighboors(m, :);
%             disp(agents);
            omega(n, min(agents)) = p * (zeta(max(agents)) - zeta(n)) + omega(n, min(agents));
            omega(n, max(agents)) = p * (zeta(n) - zeta(min(agents))) + omega(n, max(agents));
        end
        
    % 近傍のエージェントの数が2つの場合
    else
        agents = find(formation_conf(n, :) > 0); % 自分の近傍のエージェントの番号の配列を取得
        omega(n, min(agents)) = p * (zeta(max(agents)) - zeta(n)); % wij = p * (zeta_k - zeta_i)
        omega(n, max(agents)) = p * (zeta(n) - zeta(min(agents))); % wik = p * (zeta_i - zeta_j)
    end
end

for n = 1:length(zeta)
    for m = 1:length(zeta)
       if n == m
           L(n, m) = - sum(omega(n, :));
       else
           L(n, m) = omega(n, m);
       end
    end
end

%% グラフラプラシアンの固有値操作、安定化

K = eye(length(zeta));
sigma = zeros(length(zeta));
I = eye(length(zeta)-2);
Fk = [];
G = [];
% 右複素平面に固有値を持つ対角行列
for n = 1:length(zeta)-2
    sigma(n, n) = rand() + j * rand();
end
[u, s, v] = svd(L);
v2 = v(:, 1:16)';
u2 = u(:, 1:16);
s2 = s(1:16, 1:16); 

U = u2 * s2;
V = v2;
% K = [sigma(1:16, 1:16) zeros(16, 2); zeros(2, 16) eye(2)];
for n = 1:100
    
    Ak = V * K * U;
    
    % ニュートン反復法で安定化行列を求める
    for q = 1:length(zeta) - 2
        Fk(q) = det(Ak + sigma(q, q) * I); 
    end

    for q = 1: length(zeta) - 2
        if Fk(q) ~= 0 + 0j
            for p = 1: length(zeta) - 2
                G(q, p) = Fk(q) * trace( pinv(Ak + sigma(q, q) * I) * V(:, p) * U(p, :));
            end
        else
            for p = 1: length(zeta) - 2
                G(q, p) = trace( adj(Ak + sigma(q, q) * I) * V(:, p) * U(p, :));
            end
        end
    end
    K = [K(1:16, 1:16) - (pinv(G) * Fk') zeros(16, 2); zeros(2, 16) eye(2)];

    
    if n == 100
       Fk
    end
end







%%