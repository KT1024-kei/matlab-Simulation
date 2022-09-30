Position = out.position.signals.values;
Attitude = out.Attitude.signals.values;
Cable = out.cablevector.signals.values;

p = paramdef;

p1 = Position(1, :) + Attitude(:,:,1) * [0.5;0.5;0];
p2 = Position(1, :) + Attitude(:,:,1) * [-0.5;0.5;0];
p3 = Position(1, :) + Attitude(:,:,1) * [-0.5;-0.5;0];
p4 = Position(1, :) + Attitude(:,:,1) * [0.5;-0.5;0];


L = p.L;
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
    figure(1)
    plot3(Pall(:, 1), Pall(:, 2), -Pall(:, 3), 'o', 'MarkerSize', 15)
    hold on
    plot3(Pall(1:2, 1), Pall(1:2, 2), -Pall(1:2, 3), '-', 'LineWidth', 5);
    plot3(Pall(3:4, 1), Pall(3:4, 2), -Pall(3:4, 3), '-', 'LineWidth', 5);
    plot3([P(1);PL(1)], [P(2);PL(2)], [-P(3);-PL(3)], '-', 'LineWidth', 2);
    grid on;
    xlim([-1, 3]);
    ylim([-1, 3]);
    zlim([-1, 3]);
    hold off
    
end