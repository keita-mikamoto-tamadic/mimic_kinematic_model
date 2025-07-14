function [f_b, tau] = inverse_dynamics_unit(q, dq, ddq, model)
% 単位ベクトル法による逆動力学計算
% 微分運動学を用いた速度・加速度の伝播

% 運動学計算
kin = compute_kinematics(q, model);
g_vec = model.g_vec;
link_num = model.link_num;

% ベース部分の運動
p_b = kin.p_b;
R_b = kin.R_b;
v_b = dq(1:3);
omega_b = dq(4:6);
dv_b = ddq(1:3);
domega_b = ddq(4:6);

% 重力を考慮（加速度から重力を引く）
dv_b = dv_b - g_vec;

% ベース部分の慣性力
m_b = model.m_b;
Ib = model.Ib;
I_b_world = R_b * Ib * R_b';
f_b = m_b * dv_b;
t_b = I_b_world * domega_b + cross(omega_b, I_b_world * omega_b);

% 各リンクの微分運動学による速度・加速度計算
[vel_R, acc_R] = compute_differential_kinematics(q, dq, ddq, kin, model, 'R');
[vel_L, acc_L] = compute_differential_kinematics(q, dq, ddq, kin, model, 'L');

% 右脚の慣性力計算
f_b_sum = f_b;
t_b_sum = t_b;
tau_R = zeros(link_num, 1);

for i = 1:link_num
    R = kin.rot_R_total{i+1};
    c = kin.c_links_R{i};
    I = R * model.I_list_R{i} * R';
    m = model.m_link_R(i);
    
    % 微分運動学による速度・加速度
    omega_link = vel_R.omega{i};
    dv_link = acc_R.dv{i};
    domega_link = acc_R.domega{i};
    
    % 重心加速度（重力を考慮）
    dv_cog = dv_link - g_vec;
    
    % 慣性力の計算
    f = m * dv_cog;
    t = I * domega_link + cross(omega_link, I * omega_link) + cross(c - kin.p_b, f);
    
    f_b_sum = f_b_sum + f;
    t_b_sum = t_b_sum + t;
    
    % 関節軸方向の投影
    joint_axis = R(:,2);  % Y軸周り回転
    tau_R(i) = joint_axis' * t;
end

% 左脚の慣性力計算
tau_L = zeros(link_num, 1);

for i = 1:link_num
    R = kin.rot_L_total{i+1};
    c = kin.c_links_L{i};
    I = R * model.I_list_L{i} * R';
    m = model.m_link_L(i);
    
    % 微分運動学による速度・加速度
    omega_link = vel_L.omega{i};
    dv_link = acc_L.dv{i};
    domega_link = acc_L.domega{i};
    
    % 重心加速度（重力を考慮）
    dv_cog = dv_link - g_vec;
    
    % 慣性力の計算
    f = m * dv_cog;
    t = I * domega_link + cross(omega_link, I * omega_link) + cross(c - kin.p_b, f);
    
    f_b_sum = f_b_sum + f;
    t_b_sum = t_b_sum + t;
    
    % 関節軸方向の投影
    joint_axis = R(:,2);  % Y軸周り回転
    tau_L(i) = joint_axis' * t;
end

f_b = [f_b_sum; t_b_sum];
tau = [tau_R; tau_L];
end