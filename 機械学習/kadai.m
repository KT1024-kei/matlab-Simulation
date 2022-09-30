%% 初期設定
clc; clear; close all;

n = 30;                            % データ点の数 
xmin = -5;                         % xの下限
xmax = 5;                          % xの上限
x = xmin + xmax*2*rand(n, 1);      % xminからxmaxの間でデータ点を作成
ymin = -1;                         % yの下限
ymax = 1;                          % yの上限
y = ymin + ymax*2*rand(n, 1);      % yminからymaxの間でデータ点を作成

%% 訓練データの生成

% 分類する境界線の関数は x/(1 + x^2) 
data1 = [];
data2 = [];
for i=1:n
   if y(i) > x(i)/(1 + x(i)^2) 
       data1 = [data1; [x(i) y(i)]];
   else
       data2 = [data2; [x(i) y(i)]];
   end
end

%% 訓練データと境界線の描画
figure(1);
X = linspace(xmin, xmax, 100);

hold on 
scatter(data1(:, 1), data1(:, 2), 10, 'r');
scatter(data2(:, 1), data2(:, 2), 10, 'b');
plot(X,  X./(1 + X.^2));
xlim([xmin, xmax])
ylim([ymin, ymax])
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', '$f(x,y)=0$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);

%% 学習アルゴリズム

data = [data1;data2];
Lamda = diag([-ones(1, length(data1(:, 1))) ones(1, length(data2(:, 1)))]);
K = zeros(n);
% カーネル関数はGauss関数
for i = 1:n
   for j = 1:n
        K(i, j) = exp(-((data(i, 1) - data(j, 1))^2 + (data(i, 2) - data(j, 2))^2)/2);         % Gauss関数
   end
end

H = [K zeros(n, 1); zeros(1, n+1)];
f = zeros(n + 1, 1);
A = -Lamda*[K ones(n, 1)];
b = -ones(n, 1);

c = quadprog(H, f, A, b);

%% 学習結果の描画
figure(2);
scatter(data1(:, 1), data1(:, 2), 10, 'r');
hold on;
scatter(data2(:, 1), data2(:, 2), 10, 'b');


fimplicit(@(x,y) c(1:n)'* exp( -((x - data(:, 1)).^2 + (y - data(:, 2)).^2)/2)+ c(n+1) , 'Linestyle', '-', 'Color', 'green', 'LineWidth', 1);

xlim([xmin-1, xmax+1])
ylim([ymin-1, ymax+1])
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', '$f(x,y)=0$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);



