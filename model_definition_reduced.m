function model_reduced = model_definition_reduced()
% 制約を考慮した削減されたモデル定義
% 制約: theta2 = 2*theta1 により自由度を削減
% 
% 出力:
%   model_reduced: 削減されたモデル構造体（10自由度）
%   - ベース部分: 6自由度（変更なし）
%   - 関節部分: 4自由度（各脚でtheta1, wheel のみ）

% 元のモデルパラメータを取得
param_definition;

% 基本パラメータ
model_reduced.g_vec = [0; 0; -9.81];
model_reduced.link_num = 3;  % 各脚のリンク数（股、膝、車輪）
model_reduced.wheel_radius = wheel_radius;

% 質量パラメータ
model_reduced.m_b = m(1);
model_reduced.m_link_R = m(2:4);
model_reduced.m_link_L = m(5:7);

% 慣性パラメータ
model_reduced.Ib = Ib;
model_reduced.I_list_R = {I1, I2, I_wheel_R};
model_reduced.I_list_L = {I3, I4, I_wheel_L};

% 重心位置
model_reduced.c_b = [cogx_b; cogy_b; cogz_b];
model_reduced.c_links_R = { [l1_cogx; l1_cogy; l1_cogz]; [l2_cogx; l2_cogy; l2_cogz]; [wR_cogx; wR_cogy; wR_cogz] };
model_reduced.c_links_L = { [l3_cogx; l3_cogy; l3_cogz]; [l4_cogx; l4_cogy; l4_cogz]; [wL_cogx; wL_cogy; wL_cogz] };

% リンク位置
model_reduced.p_links_R = { [l1_x; l1_y; l1_z]; [l2_x; l2_y; l2_z]; [wR_x; wR_y; wR_z] };
model_reduced.p_links_L = { [l3_x; l3_y; l3_z]; [l4_x; l4_y; l4_z]; [wL_x; wL_y; wL_z] };

% 回転行列生成関数
model_reduced.ry = @(theta) rodrigues([0; 1; 0], theta);
model_reduced.r_base = @(phi, theta, psi) rodrigues([1; 0; 0], phi) * rodrigues([0; 1; 0], theta) * rodrigues([0; 0; 1], psi);

% 削減された自由度数
model_reduced.dof = 10;  % ベース6 + 関節4

% 制約関係を示す係数行列
% 完全な関節角度 = C_expand * 削減された関節角度
% [theta1_R; theta2_R; wheel_R; theta1_L; theta2_L; wheel_L] = C_expand * [theta1_R; wheel_R; theta1_L; wheel_L]
model_reduced.C_expand = [
    1, 0, 0, 0;  % theta1_R
    2, 0, 0, 0;  % theta2_R = 2*theta1_R
    0, 1, 0, 0;  % wheel_R
    0, 0, 1, 0;  % theta1_L
    0, 0, 2, 0;  % theta2_L = 2*theta1_L
    0, 0, 0, 1   % wheel_L
];

% 削減された関節角度から完全な関節角度への変換関数
model_reduced.expand_joints = @(theta_reduced) model_reduced.C_expand * theta_reduced;

% 制約力の係数行列（theta2軸のトルクをtheta1軸に変換）
% tau_theta1_equivalent = tau_theta1 + 2*tau_theta2 (制約による力の伝播)
model_reduced.C_torque = [
    1, 2, 0;  % 右脚: tau1_equiv = tau1 + 2*tau2
    0, 0, 1;  % 右脚: tau_wheel = tau_wheel
    1, 2, 0;  % 左脚: tau1_equiv = tau1 + 2*tau2
    0, 0, 1   % 左脚: tau_wheel = tau_wheel
];

% 削減されたトルク入力から等価トルクへの変換関数
model_reduced.expand_torque = @(tau_reduced) model_reduced.C_torque * reshape(tau_reduced, 2, 2)';

% 状態変数のインデックス（削減されたモデル）
model_reduced.idx_base_pos = 1:3;
model_reduced.idx_base_rot = 4:6;
model_reduced.idx_joints = 7:10;
model_reduced.idx_theta1_R = 7;
model_reduced.idx_wheel_R = 8;
model_reduced.idx_theta1_L = 9;
model_reduced.idx_wheel_L = 10;

% 元のモデルとの互換性のための関数
model_reduced.to_full_model = @(q_reduced, dq_reduced) expand_to_full_states(q_reduced, dq_reduced, model_reduced);
model_reduced.from_full_model = @(q_full, dq_full) extract_reduced_states(q_full, dq_full);

end

function [q_full, dq_full] = expand_to_full_states(q_reduced, dq_reduced, model_reduced)
% 削減された状態を完全な状態に拡張
base_q = q_reduced(1:6);
base_dq = dq_reduced(1:6);

joints_q_reduced = q_reduced(7:10);
joints_dq_reduced = dq_reduced(7:10);

% 制約関係を使用して完全な関節角度を計算
joints_q_full = model_reduced.C_expand * joints_q_reduced;
joints_dq_full = model_reduced.C_expand * joints_dq_reduced;

q_full = [base_q; joints_q_full];
dq_full = [base_dq; joints_dq_full];
end