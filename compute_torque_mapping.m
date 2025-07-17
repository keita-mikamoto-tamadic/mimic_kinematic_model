function [tau_full, tau_reduced_equiv] = compute_torque_mapping(tau_reduced, q_reduced, model_reduced)
% 制約を考慮したトルク変換・統合
% 制約: theta2 = -2*theta1 による力の伝播を考慮
%
% 入力:
%   tau_reduced: 削減されたトルク入力 [tau1_R, tau_wheel_R, tau1_L, tau_wheel_L] (4x1)
%   q_reduced: 削減された一般化座標 (10x1)
%   model_reduced: 削減されたモデル構造体
%
% 出力:
%   tau_full: 完全なトルク入力 [tau1_R, tau2_R, tau_wheel_R, tau1_L, tau2_L, tau_wheel_L] (6x1)
%   tau_reduced_equiv: 削減された等価トルク（仮想仕事原理による）(4x1)

% 削減されたトルクの分解
tau1_R = tau_reduced(1);
tau_wheel_R = tau_reduced(2);
tau1_L = tau_reduced(3);
tau_wheel_L = tau_reduced(4);

% 制約関係の解析
% 制約: theta2 = -2*theta1
% 制約の微分: dtheta2 = -2*dtheta1
% 制約の二階微分: ddtheta2 = -2*ddtheta1

% 仮想仕事原理による等価トルク計算
% 仮想変位: δtheta1, δtheta2 = -2*δtheta1
% 仮想仕事: δW = tau1*δtheta1 + tau2*δtheta2 = tau1*δtheta1 + tau2*(-2*δtheta1) = (tau1 - 2*tau2)*δtheta1
% 等価トルク: tau_equiv = tau1 - 2*tau2

% 制約による力の分配
% 削減されたトルクを完全なトルクに分配
% 膝トルクの2次関数による分配
% 膝トルク = 0.000718*theta1^2 + (-0.006939)*theta1 + 0.990022

% 関節角度の取得
theta1_R = q_reduced(7); % 削減された状態でのtheta1_R
theta1_L = q_reduced(9); % 削減された状態でのtheta1_L

% 膝トルクの2次関数による計算
% theta2 = -2*theta1制約では、theta1の符号を考慮してトルクを計算
% 元の膝トルク式を符号反転に対応: tau2 = f(-theta1)
tau2_R = 0.000718*(-theta1_R)^2 + (-0.006939)*(-theta1_R) + 0.990022;
tau2_L = 0.000718*(-theta1_L)^2 + (-0.006939)*(-theta1_L) + 0.990022;

% 完全なトルク入力の構成
tau_full = [tau1_R; tau2_R; tau_wheel_R; tau1_L; tau2_L; tau_wheel_L];

% より物理的に正確な等価トルク計算
% 制約による力の伝播を考慮した分配
% 制約: theta2 = -2*theta1 では、theta1の変化がtheta2に-2倍で伝播
% 逆に、theta2軸に加えられるトルクは-2倍の効果でtheta1軸に影響

% 制約を考慮した仮想仕事原理
% theta1に対する等価トルクは、theta2軸のトルクの-2倍の効果を持つ
% tau_equiv_theta1 = tau_theta1 - 2*tau_theta2（制約による力の伝播）

% 制約による等価トルクの計算
% 膝トルクの2次関数による寄与を考慮
% tau_equiv_theta1 = tau_theta1 - 2*tau_theta2
tau_reduced_equiv = tau_reduced;
tau_reduced_equiv(1) = tau_reduced(1) - 2*tau2_R;  % 右脚の等価トルク
tau_reduced_equiv(3) = tau_reduced(3) - 2*tau2_L;  % 左脚の等価トルク

% デバッグ情報の追加
constraint_info.tau_reduced = tau_reduced;
constraint_info.tau_full = tau_full;
constraint_info.constraint_ratio = -2.0;  % theta2 = -2*theta1
constraint_info.knee_torque_R = tau2_R;
constraint_info.knee_torque_L = tau2_L;
constraint_info.theta1_R = theta1_R;
constraint_info.theta1_L = theta1_L;
constraint_info.virtual_work_consistent = true;

% 削減されたモデルに制約情報を保存（デバッグ用）
if nargout > 2
    varargout{1} = constraint_info;
end

end