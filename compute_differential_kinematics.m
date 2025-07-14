function [vel, acc] = compute_differential_kinematics(q, dq, ddq, kin, model, side)
% 微分運動学による速度・加速度の伝播計算
% 参考：梶田先生のForwardAllKinematicsを基に実装

link_num = model.link_num;

% 出力構造体の初期化
vel.omega = cell(link_num, 1);
vel.v = cell(link_num, 1);
acc.domega = cell(link_num, 1);
acc.dv = cell(link_num, 1);

% サイド選択
if strcmp(side, 'R')
    rot_total = kin.rot_R_total;
    p_links = kin.p_links_R;
    c_links = kin.c_links_R;
    dq_joints = dq(7:9);   % 右脚関節角速度
    ddq_joints = ddq(7:9); % 右脚関節角加速度
else
    rot_total = kin.rot_L_total;
    p_links = kin.p_links_L;
    c_links = kin.c_links_L;
    dq_joints = dq(10:12);  % 左脚関節角速度
    ddq_joints = ddq(10:12); % 左脚関節角加速度
end

% ベース部分の運動
v_b = dq(1:3);
omega_b = dq(4:6);
dv_b = ddq(1:3);
domega_b = ddq(4:6);

% 各リンクの微分運動学計算
for i = 1:link_num
    % 関節軸ベクトル（Y軸周り回転）
    R_parent = rot_total{i};
    joint_axis = R_parent(:, 2);  % Y軸方向
    
    % 関節位置（重心位置）
    p_joint = c_links{i};
    
    % 空間速度ヤコビアン
    hw = joint_axis;                    % 角速度ヤコビアン
    hv = cross(p_joint - kin.p_b, hw); % 線速度ヤコビアン
    
    % 速度の計算（運動連鎖）
    if i == 1
        vel.omega{i} = omega_b + hw * dq_joints(i);
        vel.v{i} = v_b + hv * dq_joints(i);
    else
        vel.omega{i} = vel.omega{i-1} + hw * dq_joints(i);
        vel.v{i} = vel.v{i-1} + hv * dq_joints(i);
    end
    
    % 加速度の計算（コリオリ項を含む）
    if i == 1
        parent_omega = omega_b;
        parent_v = v_b;
        parent_domega = domega_b;
        parent_dv = dv_b;
    else
        parent_omega = vel.omega{i-1};
        parent_v = vel.v{i-1};
        parent_domega = acc.domega{i-1};
        parent_dv = acc.dv{i-1};
    end
    
    % 時間微分項（コリオリ項）
    dhv = cross(parent_omega, hv) + cross(parent_v, hw);
    dhw = cross(parent_omega, hw);
    
    % 加速度の伝播
    acc.domega{i} = parent_domega + dhw * dq_joints(i) + hw * ddq_joints(i);
    acc.dv{i} = parent_dv + dhv * dq_joints(i) + hv * ddq_joints(i);
end

end