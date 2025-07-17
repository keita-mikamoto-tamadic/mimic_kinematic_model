function visualize_robot(kin, model, model_type)
% 脚車輪型倒立振子ロボットの可視化
%
% 入力:
%   kin: 運動学計算結果構造体
%   model: モデル構造体
%   model_type: モデルタイプ ('original' または 'reduced')

if nargin < 3
    model_type = 'original';
end

% 新しいフィギュアを作成
figure('Name', 'Robot Visualization', 'Position', [100, 100, 1000, 800]);
clf;
hold on;
grid on;
axis equal;

% 床面の表示（z=0）
floor_size = 0.5;
floor_res = 0.05;
[X, Y] = meshgrid(-floor_size:floor_res:floor_size, -floor_size:floor_res:floor_size);
Z = zeros(size(X));

% 床面を薄いグレーで表示
surf(X, Y, Z, 'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% 床面のグリッド線
grid_spacing = 0.1;
for i = -floor_size:grid_spacing:floor_size
    plot3([i, i], [-floor_size, floor_size], [0, 0], 'k-', 'LineWidth', 0.5, 'Color', [0.8 0.8 0.8]);
    plot3([-floor_size, floor_size], [i, i], [0, 0], 'k-', 'LineWidth', 0.5, 'Color', [0.8 0.8 0.8]);
end

% 原点の表示
plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% ベースの描画
base_pos = kin.p_b;
plot3(base_pos(1), base_pos(2), base_pos(3), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'LineWidth', 2);

% ベースの重心位置の描画
if isfield(kin, 'R_b') && isfield(model, 'c_b')
    cog_b = base_pos + kin.R_b * model.c_b;
    plot3(cog_b(1), cog_b(2), cog_b(3), 'kx', 'MarkerSize', 10, 'LineWidth', 3);
end

% 座標軸の描画（ベース）
axis_length = 0.08;
if isfield(kin, 'R_b')
    drawCoordinateFrame(base_pos, kin.R_b, axis_length);
end

% 右脚リンクの描画
if isfield(kin, 'p_links_R') && isfield(model, 'link_num')
    p_prev = base_pos;
    for i = 1:model.link_num
        p_current = kin.p_links_R{i};
        
        % リンクを太い線で表示
        plot3([p_prev(1), p_current(1)], ...
              [p_prev(2), p_current(2)], ...
              [p_prev(3), p_current(3)], ...
              'r-', 'LineWidth', 4);
        
        % ジョイントを円で表示
        plot3(p_current(1), p_current(2), p_current(3), ...
              'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);
        
        % 重心位置の描画
        if isfield(kin, 'c_links_R')
            c = kin.c_links_R{i};
            plot3(c(1), c(2), c(3), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
        end
        
        % 車輪の場合（最後のリンク）
        if i == model.link_num
            % 車輪の中心から床面への垂直線
            wheel_center = p_current;
            wheel_contact = [wheel_center(1), wheel_center(2), 0];
            plot3([wheel_center(1), wheel_contact(1)], ...
                  [wheel_center(2), wheel_contact(2)], ...
                  [wheel_center(3), wheel_contact(3)], ...
                  'r:', 'LineWidth', 3);
            
            % 床面接触点
            plot3(wheel_contact(1), wheel_contact(2), wheel_contact(3), ...
                  'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'yellow', 'LineWidth', 2);
            
            % 車輪の円を描画
            drawWheel(wheel_center, 0.038975, 'r');
        end
        
        % 座標軸の描画
        if isfield(kin, 'rot_R_total')
            drawCoordinateFrame(p_current, kin.rot_R_total{i+1}, axis_length);
        end
        
        p_prev = p_current;
    end
end

% 左脚リンクの描画
if isfield(kin, 'p_links_L') && isfield(model, 'link_num')
    p_prev = base_pos;
    for i = 1:model.link_num
        p_current = kin.p_links_L{i};
        
        % リンクを太い線で表示
        plot3([p_prev(1), p_current(1)], ...
              [p_prev(2), p_current(2)], ...
              [p_prev(3), p_current(3)], ...
              'b-', 'LineWidth', 4);
        
        % ジョイントを円で表示
        plot3(p_current(1), p_current(2), p_current(3), ...
              'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'LineWidth', 2);
        
        % 重心位置の描画
        if isfield(kin, 'c_links_L')
            c = kin.c_links_L{i};
            plot3(c(1), c(2), c(3), 'bx', 'MarkerSize', 8, 'LineWidth', 2);
        end
        
        % 車輪の場合（最後のリンク）
        if i == model.link_num
            % 車輪の中心から床面への垂直線
            wheel_center = p_current;
            wheel_contact = [wheel_center(1), wheel_center(2), 0];
            plot3([wheel_center(1), wheel_contact(1)], ...
                  [wheel_center(2), wheel_contact(2)], ...
                  [wheel_center(3), wheel_contact(3)], ...
                  'b:', 'LineWidth', 3);
            
            % 床面接触点
            plot3(wheel_contact(1), wheel_contact(2), wheel_contact(3), ...
                  'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'yellow', 'LineWidth', 2);
            
            % 車輪の円を描画
            drawWheel(wheel_center, 0.038975, 'b');
        end
        
        % 座標軸の描画
        if isfield(kin, 'rot_L_total')
            drawCoordinateFrame(p_current, kin.rot_L_total{i+1}, axis_length);
        end
        
        p_prev = p_current;
    end
end

% 軸の設定
xlabel('X [m]', 'FontSize', 12);
ylabel('Y [m]', 'FontSize', 12);
zlabel('Z [m]', 'FontSize', 12);

% 軸の範囲を設定（ロボットが完全に見えるように）
xlim([-0.4, 0.4]);
ylim([-0.4, 0.4]);
zlim([-0.05, 0.4]);  % 床面より少し下から表示

% 視点を調整
view(45, 15);

% タイトルとモデル情報
if strcmp(model_type, 'reduced')
    title('脚車輪型倒立振子ロボット - 削減モデル (theta2 = -2*theta1)', 'FontSize', 14, 'FontWeight', 'bold');
else
    title('脚車輪型倒立振子ロボット - 元モデル', 'FontSize', 14, 'FontWeight', 'bold');
end

% 凡例
legend_items = {'床面', '原点', 'ベース', '右脚リンク', '右脚関節', '左脚リンク', '左脚関節', '車輪接触点'};
legend(legend_items, 'Location', 'eastoutside', 'FontSize', 10);

% グラフの見やすさを調整
ax = gca;
ax.Clipping = 'off';
ax.GridAlpha = 0.3;
ax.GridLineStyle = '-';

hold off;
end

function drawCoordinateFrame(origin, R, length)
% 座標系を描画
if size(R, 1) == 3 && size(R, 2) == 3
    % X軸（赤）
    x_end = origin + R(:, 1) * length;
    plot3([origin(1), x_end(1)], [origin(2), x_end(2)], [origin(3), x_end(3)], ...
          'r-', 'LineWidth', 2);
    
    % Y軸（緑）
    y_end = origin + R(:, 2) * length;
    plot3([origin(1), y_end(1)], [origin(2), y_end(2)], [origin(3), y_end(3)], ...
          'g-', 'LineWidth', 2);
    
    % Z軸（青）
    z_end = origin + R(:, 3) * length;
    plot3([origin(1), z_end(1)], [origin(2), z_end(2)], [origin(3), z_end(3)], ...
          'b-', 'LineWidth', 2);
end
end

function drawWheel(center, radius, color)
% 車輪を円として描画（XZ平面、Y軸周りに回転）
theta = linspace(0, 2*pi, 50);
circle_x = center(1) + radius * cos(theta);
circle_y = center(2) * ones(size(theta));  % Y座標は一定
circle_z = center(3) + radius * sin(theta);

plot3(circle_x, circle_y, circle_z, color, 'LineWidth', 2);

% 車輪の中心軸（Y軸方向）
plot3([center(1), center(1)], [center(2)-radius/2, center(2)+radius/2], ...
      [center(3), center(3)], [color, '-'], 'LineWidth', 3);

% 車輪の スポーク（十字）
spoke_length = radius * 0.7;
% 水平スポーク
plot3([center(1)-spoke_length, center(1)+spoke_length], ...
      [center(2), center(2)], [center(3), center(3)], ...
      [color, '-'], 'LineWidth', 2);
% 垂直スポーク
plot3([center(1), center(1)], [center(2), center(2)], ...
      [center(3)-spoke_length, center(3)+spoke_length], ...
      [color, '-'], 'LineWidth', 2);
end