function [ddq_reduced, lambda] = forward_dynamics_constrained_reduced(q_reduced, dq_reduced, tau_reduced, model_reduced)
% 制約を考慮した制約付き順動力学計算
% 制約: theta2 = -2*theta1 および床接触制約を考慮
%
% 入力:
%   q_reduced: 削減された一般化座標 [base(6), theta1_R, wheel_R, theta1_L, wheel_L] (10x1)
%   dq_reduced: 削減された一般化速度 (10x1)
%   tau_reduced: 削減されたトルク入力 [tau1_R, tau_wheel_R, tau1_L, tau_wheel_L] (4x1)
%   model_reduced: 削減されたモデル構造体
%
% 出力:
%   ddq_reduced: 削減された一般化加速度 (10x1)
%   lambda: ラグランジュ乗数（床反力） (2x1)

% 削減されたトルクから完全なトルクへの変換
[tau_full, ~] = compute_torque_mapping(tau_reduced, q_reduced, model_reduced);

% 削減された状態を完全な状態に拡張
[q_full, dq_full] = model_reduced.to_full_model(q_reduced, dq_reduced);

% 元のモデルを取得
model_original = model_definition_numeric();

% 元の制約付き順動力学を使用
[ddq_full, lambda] = forward_dynamics_constrained(q_full, dq_full, tau_full, model_original);

% 制約を考慮した削減された加速度の抽出
% 制約: theta2 = -2*theta1 の二階微分: ddtheta2 = -2*ddtheta1

% ベース部分の加速度
ddq_base = ddq_full(1:6);

% 関節部分の加速度（制約を考慮）
ddtheta1_R = ddq_full(7);
ddtheta2_R = ddq_full(8);  % 制約チェック用
ddwheel_R = ddq_full(9);
ddtheta1_L = ddq_full(10);
ddtheta2_L = ddq_full(11);  % 制約チェック用
ddwheel_L = ddq_full(12);

% 制約の満足度確認
constraint_error_R = abs(ddtheta2_R - (-2*ddtheta1_R));
constraint_error_L = abs(ddtheta2_L - (-2*ddtheta1_L));
tolerance = 1e-6;

if constraint_error_R > tolerance
    warning('右脚の加速度制約条件が満たされていません: |ddtheta2_R - (-2*ddtheta1_R)| = %.6f', constraint_error_R);
end

if constraint_error_L > tolerance
    warning('左脚の加速度制約条件が満たされていません: |ddtheta2_L - (-2*ddtheta1_L)| = %.6f', constraint_error_L);
end

% 削減された関節加速度
ddq_joints_reduced = [ddtheta1_R; ddwheel_R; ddtheta1_L; ddwheel_L];

% 削減された一般化加速度
ddq_reduced = [ddq_base; ddq_joints_reduced];

end