function vel = compute_velocity_reduced(q_reduced, dq_reduced, model_reduced)
% 制約を考慮した速度計算
% 制約: theta2 = 2*theta1 を考慮して削減された状態から速度を計算
%
% 入力:
%   q_reduced: 削減された一般化座標 [base(6), theta1_R, wheel_R, theta1_L, wheel_L] (10x1)
%   dq_reduced: 削減された一般化速度 (10x1)
%   model_reduced: 削減されたモデル構造体
%
% 出力:
%   vel: 速度計算結果構造体

% ベース部分の抽出
x_b = q_reduced(1:3);
eul_b = q_reduced(4:6);
dx_b = dq_reduced(1:3);
deul_b = dq_reduced(4:6);

% 削減された関節角度・角速度の抽出
theta1_R = q_reduced(7);
wheel_R = q_reduced(8);
theta1_L = q_reduced(9);
wheel_L = q_reduced(10);

dtheta1_R = dq_reduced(7);
dwheel_R = dq_reduced(8);
dtheta1_L = dq_reduced(9);
dwheel_L = dq_reduced(10);

% 制約関係を使用して完全な関節角速度を計算
theta2_R = 2 * theta1_R;
theta2_L = 2 * theta1_L;
dtheta2_R = 2 * dtheta1_R;
dtheta2_L = 2 * dtheta1_L;

% ベース姿勢とその微分
R_b = model_reduced.r_base(eul_b(1), eul_b(2), eul_b(3));
p_b = x_b;

% オイラー角の微分から角速度ベクトルへの変換
% 簡略化のため、小角度近似を使用
% 厳密な計算が必要な場合は、オイラー角変換行列を使用
omega_b = deul_b;  % 小角度近似

% 各関節の角速度
omega_joints_R = [dtheta1_R; dtheta2_R; dwheel_R];
omega_joints_L = [dtheta1_L; dtheta2_L; dwheel_L];

% 運動学計算
kin = compute_kinematics_reduced(q_reduced, model_reduced);

% リンク速度の計算（単純化された実装）
% 完全な実装には、微分運動学の連鎖則を使用
v_links_R = cell(model_reduced.link_num, 1);
v_links_L = cell(model_reduced.link_num, 1);
omega_links_R = cell(model_reduced.link_num, 1);
omega_links_L = cell(model_reduced.link_num, 1);

% 各リンクの速度と角速度（簡略化実装）
for i = 1:model_reduced.link_num
    % 右脚
    omega_links_R{i} = omega_b + [0; omega_joints_R(i); 0];  % y軸回転
    v_links_R{i} = dx_b;  % 簡略化：並進速度はベース速度のみ
    
    % 左脚
    omega_links_L{i} = omega_b + [0; omega_joints_L(i); 0];  % y軸回転
    v_links_L{i} = dx_b;  % 簡略化：並進速度はベース速度のみ
end

% 速度結果の構造体作成
vel.dx_b = dx_b;
vel.omega_b = omega_b;
vel.v_links_R = v_links_R;
vel.v_links_L = v_links_L;
vel.omega_links_R = omega_links_R;
vel.omega_links_L = omega_links_L;

% 削減された状態の保存
vel.dq_reduced = dq_reduced;
vel.dtheta1_R = dtheta1_R;
vel.dtheta2_R = dtheta2_R;
vel.dwheel_R = dwheel_R;
vel.dtheta1_L = dtheta1_L;
vel.dtheta2_L = dtheta2_L;
vel.dwheel_L = dwheel_L;

% 制約関係の速度レベルでの確認
vel.constraint_satisfied = true;
vel.dconstraint_error_R = 0;  % dtheta2_R - 2*dtheta1_R = 0
vel.dconstraint_error_L = 0;  % dtheta2_L - 2*dtheta1_L = 0

end