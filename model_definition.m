% パラメータロード
param_definition

%% シンボリック定義
syms t_symbol           % 独立変数（時間）

% フローティングベース座標・姿勢を時間関数に
syms x_b(t_symbol) y_b(t_symbol) z_b(t_symbol)
syms phi(t_symbol) theta(t_symbol) psi(t_symbol)

link_num = 3;

% 右脚関節
syms theta1(t_symbol)
theta2 = -2 * theta1(t_symbol);   % 右ひざ
syms theta_wheel_R(t_symbol)

% 左脚関節
syms theta3(t_symbol)
theta4 = -2 * theta3(t_symbol);   % 左ひざ
syms theta_wheel_L(t_symbol)

%% ボディ回転
Rx_b = [1 0 0;
        0 cos(phi(t_symbol)) -sin(phi(t_symbol));
        0 sin(phi(t_symbol))  cos(phi(t_symbol))];

Ry_b = [cos(theta(t_symbol)) 0 sin(theta(t_symbol));
        0 1 0;
       -sin(theta(t_symbol)) 0 cos(theta(t_symbol))];

Rz_b = [cos(psi(t_symbol)) -sin(psi(t_symbol)) 0;
        sin(psi(t_symbol))  cos(psi(t_symbol)) 0;
        0 0 1];

R_b = Rz_b * Ry_b * Rx_b;

%% y軸回転関数（t_symbol依存引数）
Ry = @(th) [cos(th) 0 sin(th);
             0 1 0;
            -sin(th) 0 cos(th)];

%% 相対原点位置と回転行列
% ボディ位置
p_b = [x_b(t_symbol); y_b(t_symbol); z_b(t_symbol)];
c_b = [cogx_b; cogy_b; cogz_b];

% リンク1 R
p_l1 = [l1_x; l1_y; l1_z];
c_l1 = [l1_cogx; l1_cogy; l1_cogz];
R_l1 = Ry(theta1(t_symbol));

% リンク2 R
p_l2 = [l2_x; l2_y; l2_z];
c_l2 = [l2_cogx; l2_cogy; l2_cogz];
R_l2 = Ry(theta2);

% ホイール R
wheel_R = [wR_x; wR_y; wR_z];
c_wheel_R = [wR_cogx; wR_cogy; wR_cogz];
R_wheel_R = Ry(theta_wheel_R(t_symbol));

% リンクRまとめ
p_links_R = {p_l1, p_l2, wheel_R};
c_links_R = {c_l1, c_l2, c_wheel_R};
rot_R = {R_l1, R_l2, R_wheel_R};

% 左脚リンク
p_l3 = [l3_x; l3_y; l3_z];
c_l3 = [l3_cogx; l3_cogy; l3_cogz];
R_l3 = Ry(theta3(t_symbol));

p_l4 = [l4_x; l4_y; l4_z];
c_l4 = [l4_cogx; l4_cogy; l4_cogz];
R_l4 = Ry(theta4);

% ホイール L
wheel_L = [wL_x; wL_y; wL_z];
c_wheel_L = [wL_cogx; wL_cogy; wL_cogz];
R_wheel_L = Ry(theta_wheel_L(t_symbol));

% 左脚リンクまとめ
p_links_L = {p_l3, p_l4, wheel_L};
c_links_L = {c_l3, c_l4, c_wheel_L};
rot_L = {R_l3, R_l4, R_wheel_L};


% ボディ位置
p_w_b = p_b;
rot_base = R_b;
c_w_b = p_b + rot_base * c_b;


% 各リンクまでの回転行列の積算(ボディの姿勢を含めるため1行多くなる)
rot_R_total = cell(link_num+1, 1);
rot_R_total{1} = rot_base;
rot_L_total = cell(link_num+1, 1);
rot_L_total{1} = rot_base;
for i = 1:link_num
    rot_R_total{i+1} = rot_R{i} * rot_R_total{i};
    rot_L_total{i+1} = rot_L{i} * rot_L_total{i};
end


% 原点・重心
p_w_links_R = cell(link_num, 1);
c_w_links_R = cell(link_num, 1);
p_w_links_L = cell(link_num, 1);
c_w_links_L = cell(link_num, 1);

for i = 1:link_num
    if i==1
        p_w_links_R{i} = p_b + rot_R_total{i} * p_links_R{i};
        p_w_links_L{i} = p_b + rot_L_total{i} * p_links_L{i};
    else
        p_w_links_R{i} = p_w_links_R{i-1} + rot_R_total{i} * p_links_R{i};
        p_w_links_L{i} = p_w_links_L{i-1} + rot_L_total{i} * p_links_L{i};
    end
    c_w_links_R{i} = p_w_links_R{i} + rot_R_total{i+1} * c_links_R{i};
    c_w_links_L{i} = p_w_links_L{i} + rot_L_total{i+1} * c_links_L{i};
end

return


param_set = [
    x_b(t_symbol), 0;
    y_b(t_symbol), 0;
    z_b(t_symbol), 0;
    phi(t_symbol), 0;
    theta(t_symbol), 0;
    psi(t_symbol), 0;
    theta1(t_symbol), 0;
    theta3(t_symbol), pi/6;
    theta_wheel_R(t_symbol), 0;
    theta_wheel_L(t_symbol), 0;
];

% 数値代入
p_w_b_num = double(subs(p_w_b, param_set(:,1), param_set(:,2)));
c_w_b_num = double(subs(c_w_b, param_set(:,1), param_set(:,2)));
p_w_links_R_num = cellfun(@(x) double(subs(x, param_set(:,1), param_set(:,2))), p_w_links_R, 'UniformOutput', false);
c_w_links_R_num = cellfun(@(x) double(subs(x, param_set(:,1), param_set(:,2))), c_w_links_R, 'UniformOutput', false);
p_w_links_L_num = cellfun(@(x) double(subs(x, param_set(:,1), param_set(:,2))), p_w_links_L, 'UniformOutput', false);
c_w_links_L_num = cellfun(@(x) double(subs(x, param_set(:,1), param_set(:,2))), c_w_links_L, 'UniformOutput', false);

% ここから下は元のグラフ描画部そのまま
figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('脚リンク構造と重心位置（ボディ含む）');

% ボディ原点
plot3(p_w_b_num(1), p_w_b_num(2), p_w_b_num(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
% ボディ重心
plot3(c_w_b_num(1), c_w_b_num(2), c_w_b_num(3), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

% 右脚リンク
for i = 1:(length(p_w_links_R_num)-1)
    p1 = p_w_links_R_num{i};
    p2 = p_w_links_R_num{i+1};
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'r-o', 'LineWidth', 2);
end

% 右脚重心位置
for i = 1:length(c_w_links_R_num)
    p = c_w_links_R_num{i};
    plot3(p(1), p(2), p(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
end

% 左脚リンク
for i = 1:(length(p_w_links_L_num)-1)
    p1 = p_w_links_L_num{i};
    p2 = p_w_links_L_num{i+1};
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'b-o', 'LineWidth', 2);
end

% 左脚重心位置
for i = 1:length(c_w_links_L_num)
    p = c_w_links_L_num{i};
    plot3(p(1), p(2), p(3), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
end

view(3);
legend('Body Origin', 'Body CoG', 'Right Link', 'Right CoG', 'Left Link', 'Left CoG');