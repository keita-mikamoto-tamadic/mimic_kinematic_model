function [tau_required, f_ground] = inverse_dynamics_constrained(q, dq, ddq_desired, model)
% 床反力を考慮した逆動力学計算
% 指定された加速度を実現するための関節トルクと床反力を計算
%
% 入力：
%   q          : 一般化座標 [12x1]
%   dq         : 一般化速度 [12x1]
%   ddq_desired: 目標加速度 [12x1]
%   model      : ロボットモデル
%
% 出力：
%   tau_required: 必要な関節トルク [6x1]
%   f_ground    : 床反力 [2x1] (右輪Z方向, 左輪Z方向)

% 質量行列の計算
n = length(q);
M = zeros(n, n);

for i = 1:n
    ddq_unit = zeros(n, 1);
    ddq_unit(i) = 1.0;
    [f_b, tau_] = inverse_dynamics_unit(q, dq, ddq_unit, model);
    M(:,i) = [f_b; tau_];
end

% 重力・コリオリ項の計算
ddq_zero = zeros(n, 1);
[f_b0, tau0] = inverse_dynamics_unit(q, dq, ddq_zero, model);
C_G = [f_b0; tau0];

% 拘束条件のヤコビアン計算
[~, J_phi] = compute_constraints(q, dq, model);

% 逆動力学方程式：M*ddq + J_phi'*lambda = Q
% Q = [f_external; tau_joint] + C_G
% ここで f_external = 0（床反力以外の外力なし）

% 必要な一般化力の計算
Q_needed = M * ddq_desired + C_G;

% ベース部分の力・モーメント（床反力による）
f_base_needed = Q_needed(1:6);

% 関節トルク
tau_required = Q_needed(7:12);

% 床反力の計算
% J_phi' * lambda = f_base_needed の最小二乗解
% ここで J_phi は拘束条件のヤコビアン（2x12）
% J_phi' は (12x2)、lambda は床反力 (2x1)

J_phi_base = J_phi(:, 1:6);  % ベース部分のヤコビアン (2x6)

% 床反力の計算（疑似逆行列を使用）
if rank(J_phi_base') >= 2
    f_ground = pinv(J_phi_base') * f_base_needed;
else
    warning('床反力の計算で特異点が発生しました');
    f_ground = zeros(2, 1);
end

% 床反力の物理的妥当性チェック
if any(f_ground < 0)
    warning('負の床反力が発生しました（接触が失われる可能性）');
end

% 結果の確認
f_base_actual = J_phi_base' * f_ground;
residual = norm(f_base_actual - f_base_needed);

if residual > 1e-6
    warning('床反力による力のバランスが不完全です: 残差 = %e', residual);
end

end