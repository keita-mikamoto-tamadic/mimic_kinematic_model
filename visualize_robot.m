function visualize_robot(kin, model)
    figure('Name', 'Robot Visualization');
    hold on;
    grid on;
    axis equal;
    
    % ベースの描画
    plot3(kin.p_b(1), kin.p_b(2), kin.p_b(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    % ベースの重心位置の描画
    cog_b = kin.p_b + kin.R_b * model.c_b;
    plot3(cog_b(1), cog_b(2), cog_b(3), 'kx', 'MarkerSize', 8, 'LineWidth', 2);
    
    % 座標軸の描画（ベース）
    axis_length = 0.1;
    R = kin.R_b;
    drawCoordinateFrame(kin.p_b, R, axis_length);
    
    % 右脚リンクの描画
    p_prev = kin.p_b;
    for i = 1:model.link_num
        p_current = kin.p_links_R{i};
        % リンクを線で表示
        plot3([p_prev(1), p_current(1)], ...
              [p_prev(2), p_current(2)], ...
              [p_prev(3), p_current(3)], ...
              'r-', 'LineWidth', 2);
        % ジョイントを点で表示
        plot3(p_current(1), p_current(2), p_current(3), ...
              'ro', 'MarkerSize', 8, 'LineWidth', 2);
        % 重心位置の描画
        c = kin.c_links_R{i};
        plot3(c(1), c(2), c(3), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
        
        % 座標軸の描画
        drawCoordinateFrame(p_current, kin.rot_R_total{i+1}, axis_length);
        p_prev = p_current;
    end
    
    % 左脚リンクの描画
    p_prev = kin.p_b;
    for i = 1:model.link_num
        p_current = kin.p_links_L{i};
        % リンクを線で表示
        plot3([p_prev(1), p_current(1)], ...
              [p_prev(2), p_current(2)], ...
              [p_prev(3), p_current(3)], ...
              'b-', 'LineWidth', 2);
        % ジョイントを点で表示
        plot3(p_current(1), p_current(2), p_current(3), ...
              'bo', 'MarkerSize', 8, 'LineWidth', 2);
        % 重心位置の描画
        c = kin.c_links_L{i};
        plot3(c(1), c(2), c(3), 'bx', 'MarkerSize', 8, 'LineWidth', 2);
        
        % 座標軸の描画
        drawCoordinateFrame(p_current, kin.rot_L_total{i+1}, axis_length);
        p_prev = p_current;
    end
    
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    view(45, 30);
    
    % グラフの見やすさを調整
    ax = gca;
    ax.Clipping = 'off';
    hold off;
end

function drawCoordinateFrame(origin, R, length)
    % X軸（赤）
    quiver3(origin(1), origin(2), origin(3), ...
           R(1,1)*length, R(2,1)*length, R(3,1)*length, ...
           'r', 'LineWidth', 1.5);
    % Y軸（緑）
    quiver3(origin(1), origin(2), origin(3), ...
           R(1,2)*length, R(2,2)*length, R(3,2)*length, ...
           'g', 'LineWidth', 1.5);
    % Z軸（青）
    quiver3(origin(1), origin(2), origin(3), ...
           R(1,3)*length, R(2,3)*length, R(3,3)*length, ...
           'b', 'LineWidth', 1.5);
end
