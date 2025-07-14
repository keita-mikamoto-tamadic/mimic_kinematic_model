function [tau_gravity, f_ground_static] = compute_gravity_compensation(q, model)
% 重力補償トルクの計算
% 静的平衡状態（加速度=0）で床反力から必要な関節トルクを計算
%
% 入力：
%   q     : 一般化座標 [12x1]
%   model : ロボットモデル
%
% 出力：
%   tau_gravity     : 重力補償に必要な関節トルク [6x1]
%   f_ground_static : 静的平衡時の床反力 [2x1]

% 静的平衡状態の設定
dq = zeros(12, 1);         % 速度ゼロ
ddq_desired = zeros(12, 1); % 加速度ゼロ（静的平衡）

% 逆動力学による重力補償トルクの計算
[tau_gravity, f_ground_static] = inverse_dynamics_constrained(q, dq, ddq_desired, model);

fprintf('=== 重力補償トルクの計算 ===\n');
fprintf('重力補償トルク [Nm]:\n');
fprintf('  右脚: [%7.4f, %7.4f, %7.4f]\n', tau_gravity(1:3));
fprintf('  左脚: [%7.4f, %7.4f, %7.4f]\n', tau_gravity(4:6));
fprintf('静的平衡時の床反力 [N]: [%7.4f, %7.4f]\n', f_ground_static);
fprintf('総床反力: %7.4f N（総重量: %7.4f N）\n', sum(f_ground_static), sum([model.m_b, model.m_link_R, model.m_link_L]) * 9.81);

end