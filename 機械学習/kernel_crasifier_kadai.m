%% 初期設定
clc; clear; close all;

n = 5;                            % データ点の数 
xmin = -3;                         % xの下限
xmax = 3;                          % xの上限
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
% カーネル関数は正弦波かGauss関数
for i = 1:n
   for j = 1:n
       
%         K(i, j) = sin(data(i, 1)) * sin(data(j, 1));           % 正弦波
%         K(i, j) = exp(-(data(i, 1) - data(j, 1))^2/2);         % Gauss関数
        for k = 1:7
           K(i, j) = K(i, j) +  (data(i, 1) * data(j, 1))^(k-1); % 6次関数
        end
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

% カーネル関数が正弦波の場合
% x = linspace(xmin, xmax, n);
% plot(x, c(1:n)'* sin(data(:,1))*sin(x) + c(n+1), 'Linestyle', '-', 'Color', 'red')

% カーネル関数がGauss関数or　d次関数の場合の場合

y = [];
tmp = 0;
for x = linspace(xmin, xmax, 100)
    for k = 1:n
%         tmp =  tmp + c(k) * exp(-(x - data(k,1))^2/2);    % Gauss関数
        
        for i = 1:7
           tmp = tmp + c(k) * (x - data(k, 1))^(i - 1);   % d次関数
        end
    end
    y = [y tmp + c(n+1)];
    tmp = 0;
end

x = linspace(xmin, xmax, 100);
plot(x, y + c(n+1), 'Linestyle', '-', 'Color', 'red')
xlim([xmin, xmax])
ylim([ymin-5, ymax+5])
set(gca, 'FontSize',20, 'FontName', 'Times')
legend({'$\lambda=+1$', '$\lambda=-1$', '$f(x,y)=0$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex', 'Fontsize', 20); ylabel('$y$', 'Interpreter', 'latex', 'Fontsize', 20);



