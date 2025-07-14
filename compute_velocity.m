function vel = compute_velocity(q, dq, kin, model, dt)
% 入力：
%   q   : 一般化座標 [10x1]
%   dq  : 一般化速度 [10x1]
%   kin : compute_kinematics の出力
%   model : model_definition_numeric の出力
%   dt  : 数値微分ステップ幅（例: 1e-5）
%
% 出力：
%   vel.v_b         : ボディ線速度
%   vel.omega_b     : ボディ角速度（ワールド系）
%   vel.vLinks_R/L  : 各リンク原点速度（cell）
%   vel.omegaLinks_R/L : 各リンク角速度（cell）
%   vel.vc_links_R/L   : 各リンク重心速度（cell）

vee = @(S) [S(3,2); S(1,3); S(2,1)];

% ---- ボディ速度 ----
v_b = dq(1:3);  % x_b, y_b, z_b の時間微分
eul = q(4:6);   % phi, theta, psi
deul = dq(4:6);

% 回転行列とその微分（前方差分）
R_b = kin.R_b;
eul_forward = eul + dt * deul;
R_b_forward = model.R_base(eul_forward(1), eul_forward(2), eul_forward(3));
R_dot = (R_b_forward - R_b) / dt;

omega_skew = R_dot * R_b';
omega_b = vee(omega_skew);

% ---- 初期化 ----
link_num = model.link_num;
vLinks_R = cell(link_num + 1, 1);
omegaLinks_R = cell(link_num + 1, 1);
vc_links_R = cell(link_num, 1);

vLinks_L = cell(link_num + 1, 1);
omegaLinks_L = cell(link_num + 1, 1);
vc_links_L = cell(link_num, 1);

% ---- ボディ基準 ----
vLinks_R{1} = v_b;
vLinks_L{1} = v_b;
omegaLinks_R{1} = omega_b;
omegaLinks_L{1} = omega_b;

% ---- 右脚 ----
for i = 1:link_num
    R = kin.rot_R_total{i+1};

    % 回転微分（前方差分）
    q_fwd = q;
    q_fwd(6+i) = q(6+i) + dt * dq(6+i);  % theta1, theta2, wheelR
    kin_fwd = compute_kinematics(q_fwd, model);
    R_fwd = kin_fwd.rot_R_total{i+1};
    R_dot = (R_fwd - R) / dt;

    omega_skew = R_dot * R';
    omegaLinks_R{i+1} = vee(omega_skew);

    rel_pos = R * model.p_links_R{i};
    vLinks_R{i+1} = vLinks_R{i} + hat(omegaLinks_R{i}) * rel_pos;

    rel_cog = R * model.c_links_R{i};
    vc_links_R{i} = vLinks_R{i+1} + hat(omegaLinks_R{i+1}) * rel_cog;
end

% ---- 左脚 ----
for i = 1:link_num
    R = kin.rot_L_total{i+1};

    q_fwd = q;
    q_fwd(9+i) = q(9+i) + dt * dq(9+i);  % theta3, theta4, wheelL
    kin_fwd = compute_kinematics(q_fwd, model);
    R_fwd = kin_fwd.rot_L_total{i+1};
    R_dot = (R_fwd - R) / dt;

    omega_skew = R_dot * R';
    omegaLinks_L{i+1} = vee(omega_skew);

    rel_pos = R * model.p_links_L{i};
    vLinks_L{i+1} = vLinks_L{i} + hat(omegaLinks_L{i}) * rel_pos;

    rel_cog = R * model.c_links_L{i};
    vc_links_L{i} = vLinks_L{i+1} + hat(omegaLinks_L{i+1}) * rel_cog;
end

% ---- 出力構造体 ----
vel.v_b = v_b;
vel.omega_b = omega_b;
vel.vLinks_R = vLinks_R;
vel.omegaLinks_R = omegaLinks_R;
vel.vc_links_R = vc_links_R;
vel.vLinks_L = vLinks_L;
vel.omegaLinks_L = omegaLinks_L;
vel.vc_links_L = vc_links_L;

end
