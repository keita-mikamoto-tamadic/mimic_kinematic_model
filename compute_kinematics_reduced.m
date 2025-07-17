function kin = compute_kinematics_reduced(q_reduced, model_reduced)
% 制約を考慮した運動学計算
% 制約: theta2 = -2*theta1 を考慮して削減された状態から運動学を計算
%
% 入力:
%   q_reduced: 削減された一般化座標 [base(6), theta1_R, wheel_R, theta1_L, wheel_L] (10x1)
%   model_reduced: 削減されたモデル構造体
%
% 出力:
%   kin: 運動学計算結果構造体（既存の構造体と同じ形式）

% ベース部分の抽出
x_b = q_reduced(1:3);
eul_b = q_reduced(4:6);

% 削減された関節角度の抽出
theta1_R = q_reduced(7);
wheel_R = q_reduced(8);
theta1_L = q_reduced(9);
wheel_L = q_reduced(10);

% 制約関係を使用して完全な関節角度を計算
theta2_R = -2 * theta1_R;
theta2_L = -2 * theta1_L;

% ベース姿勢の計算
R_b = model_reduced.r_base(eul_b(1), eul_b(2), eul_b(3));
p_b = x_b;

% 各関節の回転行列
Ry = model_reduced.ry;
rot_R = {Ry(theta1_R); Ry(theta2_R); Ry(wheel_R)};
rot_L = {Ry(theta1_L); Ry(theta2_L); Ry(wheel_L)};

% 累積回転行列の計算
rot_R_total = cell(model_reduced.link_num+1, 1);
rot_L_total = cell(model_reduced.link_num+1, 1);
rot_R_total{1} = R_b;
rot_L_total{1} = R_b;

for i = 1:model_reduced.link_num
    rot_R_total{i+1} = rot_R_total{i} * rot_R{i};
    rot_L_total{i+1} = rot_L_total{i} * rot_L{i};
end

% リンク位置と重心位置の計算
p_links_R = cell(model_reduced.link_num, 1);
p_links_L = cell(model_reduced.link_num, 1);
c_links_R = cell(model_reduced.link_num, 1);
c_links_L = cell(model_reduced.link_num, 1);

for i = 1:model_reduced.link_num
    if i == 1
        % 最初のリンクはベースから
        p_links_R{i} = p_b + rot_R_total{i} * model_reduced.p_links_R{i};
        p_links_L{i} = p_b + rot_L_total{i} * model_reduced.p_links_L{i};
    else
        p_links_R{i} = p_links_R{i-1} + rot_R_total{i} * model_reduced.p_links_R{i};
        p_links_L{i} = p_links_L{i-1} + rot_L_total{i} * model_reduced.p_links_L{i};
    end
    c_links_R{i} = p_links_R{i} + rot_R_total{i+1} * model_reduced.c_links_R{i};
    c_links_L{i} = p_links_L{i} + rot_L_total{i+1} * model_reduced.c_links_L{i};
end

% 運動学結果の構造体作成
kin.p_b = p_b;
kin.R_b = R_b;
kin.p_links_R = p_links_R;
kin.p_links_L = p_links_L;
kin.c_links_R = c_links_R;
kin.c_links_L = c_links_L;
kin.rot_R_total = rot_R_total;
kin.rot_L_total = rot_L_total;

% 削減された状態の保存（デバッグ用）
kin.q_reduced = q_reduced;
kin.theta1_R = theta1_R;
kin.theta2_R = theta2_R;
kin.wheel_R = wheel_R;
kin.theta1_L = theta1_L;
kin.theta2_L = theta2_L;
kin.wheel_L = wheel_L;

% 制約関係の確認
kin.constraint_satisfied = true;  % 制約は構築上自動的に満たされる
kin.constraint_error_R = 0;       % theta2_R - 2*theta1_R = 0
kin.constraint_error_L = 0;       % theta2_L - 2*theta1_L = 0

end