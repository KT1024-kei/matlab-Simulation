%% データ取得
clc;clear;close all;
path = "flocking_dance_ecbf_2022-05-30_Dc0.2_Da0.4_Kc02_Kp0.015.xlsx";
data = xlsread(path);
data = data(1:end-10, :);
n = (length(data(1, :))-1)/6;
n = 10;

px = data(1:end-4, 2:n+1)';
py = data(1:end-4, 2*n+2:3*n+2)';
pz = data(1:end-4, 4*n+2:5*n+2)';
Dc = 0.2;
Da = 0.4;


%% 位置
close all;
% plot3(px, py, pz, '.')
l = ['r' 'b' 'g' 'm' 'k'];
figure(6)
h = gca;
view(3)
hold on
for i = 1:n
    m = rem(i, length(l));
    plot3(px(i, 1), py(i, 1), pz(i, 1), 'o', 'MarkerSize', 10, 'Color', 'k', 'LineWidth', 1)
end

for i = 1:n
    m = rem(i, length(l));
    plot3(px(i, 1:end), py(i, 1:end), pz(i, 1:end), '-', 'LineWidth', 3, 'Color', l(1 + m))
end
for i = 1:n
    m = rem(i, length(l));
    plot3(px(i, end), py(i, end), pz(i, end), 'd', 'MarkerSize', 15, 'Color', 'k', 'LineWidth', 1)
end

xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]')
set(gca, 'FontSize',35, 'FontName', 'Times')
xlim([-1.2 1.2]);ylim([-1.2 1.2]);zlim([0 1.5]);
grid on
h.GridAlpha = 1.0;
hold off
%% 衝突距離
tmpmin = 10000000;
tmpmax = 0;
flag = false;
cntmin = zeros(n, n);
cntmax = zeros(n, n);

G = [0 1 0 1 0 0 0 1 0 1;
     1 0 1 0 1 0 0 0 1 0;
     0 1 0 1 0 1 0 0 0 1;
     1 0 1 0 1 0 1 0 0 0;
     0 1 0 1 0 1 0 1 0 0;
     0 0 1 0 1 0 1 0 1 0;
     0 0 0 1 0 1 0 1 0 1;
     1 0 0 0 1 0 1 0 1 0;
     0 1 0 0 0 1 0 1 0 1;
     1 0 1 0 0 0 1 0 1 0];
maxagent =zeros(1, length(px));
minagent = ones(1, length(px)) * 100;
for k = 1:length(px)
    for i = 1:n
       for j = i+1:n
           if i ~= j
               if Dc >  norm([px(i, k), py(i, k), pz(i, k)] - [px(j, k), py(j, k), pz(j, k)])
                   flag = true;
               end
               if tmpmin > norm([px(i, k), py(i, k), pz(i, k)] - [px(j, k), py(j, k), pz(j, k)])
                  tmpmin = norm([px(i, k), py(i, k), pz(i, k)] - [px(j, k), py(j, k), pz(j, k)]);
                  if tmpmin < minagent(k)
                      minagent(k) = tmpmin;
                  end
                  cntmin(i, j) = cntmin(i, j) + 1;
                  minj = j;
                  mini = i;
               end
               if tmpmax < norm([px(i, k), py(i, k), pz(i, k)] - [px(j, k), py(j, k), pz(j, k)])&& G(i, j) == 1
                  tmpmax = norm([px(i, k), py(i, k), pz(i, k)] - [px(j, k), py(j, k), pz(j, k)]);
                  cntmax(i, j) = cntmax(i, j) + 1;
                  maxj = j;
                  maxi = i;
               end
           end
       end
    end
    minagent(k) = tmpmin;
    maxagent(k) = tmpmax;
    tmpmin = 100;
    tmpmax = 0;
end
if flag
   fprintf('Collision Occur\n');
else
   fprintf('No Collision\n');
end
mindist = [];
maxdist = [];
for i = 2:length(px(1, 1:end))-1
    mindist = [mindist  norm([px(mini, i), py(mini, i), pz(mini, i)] - [px(minj, i), py(minj, i), pz(minj, i)])];
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end

figure(5)
hold on
plot(maxagent, '-', 'Color', 'b', 'LineWidth', 3)
plot(minagent, '-', 'Color', 'g', 'LineWidth', 5)
line([1,length(px(1, 1:end))], [Dc, Dc], 'Color', 'r', 'LineWidth', 3);
line([1,length(px(1, 1:end))], [Da, Da], 'Color', 'm', 'LineWidth', 3);
for k = 1:10
    for j = 2:10
        if k == j || G(k, j) == 0
        else
            for i = 2:length(px(1, 1:end))-1 
                maxdist = [maxdist  norm([px(k, i), py(k, i), pz(k, i)] - [px(j, i), py(j, i), pz(j, i)])];
            end
        end
        plot(maxdist, '-', 'Color', [.8 .8 .8], 'LineWidth', 0.1)
        maxdist = [];
    end
end

for i = 2:length(px(1, 1:end))-1
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end
plot(maxagent, '-', 'Color', 'b', 'LineWidth', 3)
plot(minagent, '-', 'Color', 'g', 'LineWidth', 5)
line([1,length(px(1, 1:end))], [Dc, Dc], 'Color', 'r', 'LineWidth', 3);
line([1,length(px(1, 1:end))], [Da, Da], 'Color', 'm', 'LineWidth', 3);
xlim([1,length(px)]); ylim([0, 3.0])
grid on
xlabel('T[s]'); ylabel('Relative Distance[m]');
set(gca, 'FontSize',25, 'FontName', 'Times')
legend({'Min$\|p_{ij} \|$', 'Max$\|p_{ij} \|$', '$D_c$' '$D_a$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
hold off