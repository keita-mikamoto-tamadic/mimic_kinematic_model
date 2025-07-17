function [f_b, tau_reduced] = inverse_dynamics_reduced(q_reduced, dq_reduced, ddq_reduced, model_reduced)
% 制約を考慮した逆動力学計算
% 制約: theta2 = 2*theta1 を考慮した削減された逆動力学
%
% 入力:
%   q_reduced: 削減された一般化座標 [base(6), theta1_R, wheel_R, theta1_L, wheel_L] (10x1)
%   dq_reduced: 削減された一般化速度 (10x1)
%   ddq_reduced: 削減された一般化加速度 (10x1)
%   model_reduced: 削減されたモデル構造体
%
% 出力:
%   f_b: ベース部分の力・モーメント (6x1)
%   tau_reduced: 削減されたトルク [tau1_R, tau_wheel_R, tau1_L, tau_wheel_L] (4x1)

% 削減された状態を完全な状態に拡張
[q_full, dq_full] = model_reduced.to_full_model(q_reduced, dq_reduced);

% 削減された加速度を完全な加速度に拡張
% 制約: theta2 = 2*theta1 の二階微分: ddtheta2 = 2*ddtheta1
ddq_base = ddq_reduced(1:6);
ddtheta1_R = ddq_reduced(7);
ddwheel_R = ddq_reduced(8);
ddtheta1_L = ddq_reduced(9);
ddwheel_L = ddq_reduced(10);

% 制約を考慮した完全な関節加速度
ddtheta2_R = 2 * ddtheta1_R;
ddtheta2_L = 2 * ddtheta1_L;

% 完全な一般化加速度
ddq_full = [ddq_base; ddtheta1_R; ddtheta2_R; ddwheel_R; ddtheta1_L; ddtheta2_L; ddwheel_L];

% 元のモデルを取得
model_original = model_definition_numeric();

% 元の逆動力学を使用して完全なトルクを計算
[f_b, tau_full] = inverse_dynamics(q_full, dq_full, ddq_full, model_original);

% 完全なトルクを削減されたトルクに変換
% 制約による力の統合: tau_reduced = tau1 + 2*tau2（仮想仕事原理）
tau1_R = tau_full(1);
tau2_R = tau_full(2);
tau_wheel_R = tau_full(3);
tau1_L = tau_full(4);
tau2_L = tau_full(5);
tau_wheel_L = tau_full(6);

% 仮想仕事原理による統合
tau1_R_integrated = tau1_R + 2 * tau2_R;
tau1_L_integrated = tau1_L + 2 * tau2_L;

% 削減されたトルク
tau_reduced = [tau1_R_integrated; tau_wheel_R; tau1_L_integrated; tau_wheel_L];

% 動力等価性の確認（オプション）
% 削減された速度
dtheta1_R = dq_reduced(7);
dtheta1_L = dq_reduced(9);

% 元の速度
dtheta2_R = 2 * dtheta1_R;
dtheta2_L = 2 * dtheta1_L;

% 動力の確認
power_original_R = tau1_R * dtheta1_R + tau2_R * dtheta2_R;
power_original_L = tau1_L * dtheta1_L + tau2_L * dtheta2_L;
power_reduced_R = tau1_R_integrated * dtheta1_R;
power_reduced_L = tau1_L_integrated * dtheta1_L;

% 動力の一致確認
power_error_R = abs(power_original_R - power_reduced_R);
power_error_L = abs(power_original_L - power_reduced_L);
tolerance = 1e-10;

if power_error_R > tolerance
    warning('右脚の動力等価性が満たされていません: 誤差 = %.6e', power_error_R);
end

if power_error_L > tolerance
    warning('左脚の動力等価性が満たされていません: 誤差 = %.6e', power_error_L);
end

end