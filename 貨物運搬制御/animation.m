Position = out.position.signals.values;
Attitude = out.Attitude.signals.values;
Cable = out.cablevector.signals.values;

p = paramdef;

p1 = Position(1, :) + Attitude(:,:,1) * [0.5;0.5;0];
p2 = Position(1, :) + Attitude(:,:,1) * [-0.5;0.5;0];
p3 = Position(1, :) + Attitude(:,:,1) * [-0.5;-0.5;0];
p4 = Position(1, :) + Attitude(:,:,1) * [0.5;-0.5;0];


L = p.L;
h = figure(1);
hold(gca, 'on');
view(3);
prop = plot3(gca, Pall(:, 1), Pall(:, 2), -Pall(:, 3), 'o', 'MarkerSize', 15);
poll1 = plot3(gca, Pall(1:2, 1), Pall(1:2, 2), -Pall(1:2, 3), '-', 'LineWidth', 5);
poll2 = plot3(gca, Pall(3:4, 1), Pall(3:4, 2), -Pall(3:4, 3), '-', 'LineWidth', 5);
payload = plot3(gca, [P(1);PL(1)], [P(2);PL(2)], [-P(3);-PL(3)], '-', 'LineWidth', 2);
xlim([-3, 3]);
ylim([-3, 3]);
zlim([5, 12]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
hold(gca, 'off');
for i =1:length(Position)
    P = Position(i, :)';
    R = Attitude(:,:,i);
    p1 = P + R * [0.5;0.5;0];
    p2 = P + R * [-0.5;-0.5;0];
    p3 = P + R * [-0.5;0.5;0];
    p4 = P + R * [0.5;-0.5;0];
    
    PQ = [p1';p2';p3';p4'];
    PL = P + L*Cable(i, :)';
    Pall = [PQ;PL'];
    set(prop, 'XData', Pall(:, 1), 'YData', Pall(:, 2), 'ZData', -Pall(:, 3))
    hold on
    set(poll1,'XData',Pall(1:2, 1) , 'YData',Pall(1:2, 2) ,'ZData', -Pall(1:2, 3));
    set(poll2,'XData',Pall(3:4, 1) , 'YData',Pall(3:4, 2) ,'ZData', -Pall(3:4, 3));
    set(payload,'XData',[P(1);PL(1)] , 'YData',[P(2);PL(2)] ,'ZData', [-P(3);-PL(3)]);
    grid on;
    hold off
    drawnow
end