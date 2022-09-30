%% モデル
function d_state = kinematics_model2(state, input, param)

v_des = input(1);   % 速度？
delta_des = input(2); % ステアリング？

% limit 上限と下限
delta_des = max(min(delta_des, param.steer_lim), - param.steer_lim);  
v_des = max(min(v_des, param.vel_max), param.vel_min);

% x = state(1);
% y = state(2);

yaw = state(3);
delta = state(4);

v = v_des;

d_x = v * cos(yaw);
d_y = v * sin(yaw);
d_yaw = v * tan(delta) / param.wheelbase; % wheelbase: 前輪軸と後輪軸間の距離
d_delta = - (delta - delta_des) / param.tau; % 

