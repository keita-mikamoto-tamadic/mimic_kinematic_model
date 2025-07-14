function [f_b, tau] = inverse_dynamics(q, dq, ddq, model)
kin = compute_kinematics(q, model);
g_vec = model.g_vec;
link_num = model.link_num;

m_b = model.m_b;
Ib = model.Ib;

p_b = kin.p_b;
R_b = kin.R_b;
v_b = dq(1:3);
omega_b = dq(4:6);
dv_b = ddq(1:3);
domega_b = ddq(4:6);

I_b_world = R_b * Ib * R_b';
f_b = m_b * (dv_b - g_vec);
t_b = I_b_world * domega_b + cross(omega_b, I_b_world * omega_b);

f_b_sum = f_b;
t_b_sum = t_b;

tau_R = zeros(link_num, 1);
tau_L = zeros(link_num, 1);

for i = link_num:-1:1
    R = kin.rot_R_total{i+1};
    c = kin.c_links_R{i};
    I = R * model.I_list_R{i} * R';
    m = model.m_link_R(i);
    % 関節軸方向ベクトル（Y軸周り）
    joint_axis = R(:,2);
    
    % 角速度の計算（運動連鎖を考慮）
    omega_link = omega_b;
    domega_link = domega_b;
    % i番目までの関節の影響を累積
    for j = 1:i
        joint_axis_j = kin.rot_R_total{j+1}(:,2);
        omega_link = omega_link + joint_axis_j * dq(6+j);
        domega_link = domega_link + joint_axis_j * ddq(6+j);
    end
    
    % 並進加速度の計算（運動連鎖を考慮）
    dv = dv_b + cross(domega_link, c) + cross(omega_link, cross(omega_link, c));
    
    % 力とトルクの計算
    f = m * (dv - g_vec);
    t = I * domega_link + cross(omega_link, I * omega_link) + cross(c, f);
    f_b_sum = f_b_sum + f;
    t_b_sum = t_b_sum + t;
    tau_R(i) = R(:,2)' * t;
end

for i = link_num:-1:1
    R = kin.rot_L_total{i+1};
    c = kin.c_links_L{i};
    I = R * model.I_list_L{i} * R';
    m = model.m_link_L(i);
    % 関節軸方向ベクトル（Y軸周り）
    joint_axis = R(:,2);
    
    % 角速度の計算（運動連鎖を考慮）
    omega_link = omega_b;
    domega_link = domega_b;
    % i番目までの関節の影響を累積
    for j = 1:i
        joint_axis_j = kin.rot_L_total{j+1}(:,2);
        omega_link = omega_link + joint_axis_j * dq(9+j);
        domega_link = domega_link + joint_axis_j * ddq(9+j);
    end
    
    % 並進加速度の計算（運動連鎖を考慮）
    dv = dv_b + cross(domega_link, c) + cross(omega_link, cross(omega_link, c));
    
    % 力とトルクの計算
    f = m * (dv - g_vec);
    t = I * domega_link + cross(omega_link, I * omega_link) + cross(c, f);
    f_b_sum = f_b_sum + f;
    t_b_sum = t_b_sum + t;
    tau_L(i) = R(:,2)' * t;
end

f_b = [f_b_sum; t_b_sum];
tau = [tau_R; tau_L];
end
