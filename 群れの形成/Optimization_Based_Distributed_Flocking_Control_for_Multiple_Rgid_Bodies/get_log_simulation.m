%% 位置
% plot3(px, py, pz, '.')
l = ['r' 'b' 'g' 'y' 'k'];
figure(6)
view(3)
hold on
f = 2000;
for i = 1:n
     m = rem(i, length(l));
     plot3(px(i, 2:f), py(i, 2:f), pz(i, 2:f), '.', 'Color', l(1 + m), 'MarkerSize', 10)
     
end
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]')
set(gca, 'FontSize',25, 'FontName', 'Times')
xlim([-0 150]);ylim([-0 180]);zlim([-0 50]);
grid on
hold off
%% 姿勢
figure(2)
l = ['r' 'b' 'g' 'y' 'k'];
for i = 1:n
     m = rem(i, length(l));
     plot(attitude(i, 2:f), '.', 'Color', l(1 + m), 'MarkerSize', 10)
     hold on
end
xlabel('T[ms]'); ylabel('Yaw[rad]');
set(gca, 'FontSize',25, 'FontName', 'Times')
xlim([1, f])
grid on
hold off
%% 衝突距離
tmpmin = 10000000;
tmpmax = 0;
for i = 1:n
   for j = i+1:n
       if i ~= j
           if tmpmin > norm([px(i, f), py(i, f), pz(i, f)] - [px(j, f), py(j, f), pz(j, f)])
              tmpmin =  norm([px(i, f), py(i, f), pz(i, f)] - [px(j, f), py(j, f), pz(j, f)]);
              minj = j;
              mini = i;
           end
           if tmpmax < norm([px(i, f), py(i, f), pz(i, f)] - [px(j, f), py(j, f), pz(j, f)]) && G(i, j) ==1
              tmpmax = norm([px(i, f), py(i, f), pz(i, f)] - [px(j, f), py(j, f), pz(j, f)]);
              maxj = j;
              maxi = i;
           end
       end
   end
end
mindist = [];
maxdist = [];
for i = 2:length(px(1, 1:end))-1
    mindist = [mindist  norm([px(mini, i), py(mini, i), pz(mini, i)] - [px(minj, i), py(minj, i), pz(minj, i)])];
    maxdist = [maxdist  norm([px(maxi, i), py(maxi, i), pz(maxi, i)] - [px(maxj, i), py(maxj, i), pz(maxj, i)])];
end

figure(3)
plot(mindist, '-', 'Color', 'g', 'LineWidth', 6)
hold on
plot(maxdist, '-', 'Color', 'b', 'LineWidth', 6)
line([1,length(px(1, 1:f))], [Dc, Dc], 'Color', 'r', 'LineWidth', 5);
line([1,length(px(1, 1:f))], [Da, Da], 'Color', 'm', 'LineWidth', 5);
xlim([1,f])
grid on
xlabel('T[s]'); ylabel('Relative Distance[m]');
set(gca, 'FontSize', 35, 'FontName', 'Times')
legend({'Min$\|p_{ij} \|$', 'Max$\|p_{ij} \|$', '$D_c$' '$D_a$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
hold off
%% 衝突距離
tmpmin = 10000000;
tmpmax = 0;
flag = false;
cntmin = zeros(n, n);
cntmax = zeros(n, n);

maxagent =zeros(1, length(px));
minagent = ones(1, length(px)) * 1000;
for k = 2:length(px)
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
plot(maxagent(1, 2:end), '-', 'Color', 'b', 'LineWidth', 3)
plot(minagent(1, 2:end), '-', 'Color', 'g', 'LineWidth', 5)
line([1,length(px(1, 1:end))], [Dc, Dc], 'Color', 'r', 'LineWidth', 3);
line([1,length(px(1, 1:end))], [Da, Da], 'Color', 'm', 'LineWidth', 3);
for k = 1:20
    for j = 2:20
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
plot(maxagent(1, 2:end), '-', 'Color', 'b', 'LineWidth', 3)
plot(minagent(1, 2:end), '-', 'Color', 'g', 'LineWidth', 5)
line([1,length(px(1, 1:end))], [Dc, Dc], 'Color', 'r', 'LineWidth', 3);
line([1,length(px(1, 1:end))], [Da, Da], 'Color', 'm', 'LineWidth', 3);
xlim([1,length(px)]); ylim([0, 60])
grid on
xlabel('T[s]'); ylabel('Relative Distance[m]');
set(gca, 'FontSize',25, 'FontName', 'Times')
legend({'Min$\|p_{ij} \|$', 'Max$\|p_{ij} \|$', '$D_c$' '$D_a$'}, 'Location', 'NorthEast', 'NumColumns', 3, 'Interpreter', 'latex');
hold off