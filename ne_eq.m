%% ニュートン・オイラー法による運動方程式導出 (forループ版)
param_definition
model_definition
vel

g_vec = [0; 0; -g];
link_num = 3; % 右脚/左脚 3リンク

% ---- ボディ ----
dp_b = diff(m(1) * v_b, t_symbol);

R_base = rot_base;
Ib_rot = R_base * Ib * R_base.';
dH_b = diff(Ib_rot * omegaLinks_R{1}, t_symbol);

syms F_R [3 1] real  % ボディ→右脚への反力
syms F_L [3 1] real  % ボディ→左脚への反力

r_R = p_w_links_R{1} - p_b;
r_L = p_w_links_L{1} - p_b;

eq_body     = dp_b == m(1)*g_vec - F_R - F_L;
eq_body_ang = dH_b == - cross(r_R, F_R) - cross(r_L, F_L);

% ---- 右脚 ----
I_list_R = {I1, I2, I_wheel_R};
m_idx_R  = [2,3,4];
rot_list_R = rot_R_total;
p_list_R = p_links_R;

syms F_R_out [3 link_num] real
syms F_wR_ground [3 1] real

eqs_R = cell(link_num,2);
tau_R = sym('tau_R', [link_num,1]);  % 各関節のトルク

for i = 1:link_num
    m_i = m(m_idx_R(i));
    I_local = I_list_R{i};
    R_i = rot_list_R{i+1};
    I_rot = R_i * I_local * R_i.';

    dp_i = diff(m_i * vLinks_R{i+1}, t_symbol);
    dH_i = diff(I_rot * omegaLinks_R{i+1}, t_symbol);
    Fg_i = m_i * g_vec;

    % 上流の力
    if i == 1
        F_in = F_R;
    else
        F_in = F_R_out(:,i-1);
    end

    % 下流の力
    if i < link_num
        F_out = F_R_out(:,i);
    else
        F_out = -F_wR_ground; % 最下流：ホイールは地面反力
    end

    eqs_R{i,1} = dp_i == Fg_i + F_in - F_out;
    % ---- オイラー運動方程式＋関節射影τ ----
    % 各関節のy軸（ワールド系）
    y_axis = R_i * [0;1;0];
    % 関節点に作用するモーメントベクトル
    joint_M = -cross(p_list_R{i}, F_out);
    % 関節トルク成分（y軸射影）
    tau_R(i) = simplify(y_axis.' * joint_M);
    % 角運動量式
    eqs_R{i,2} = dH_i == joint_M + y_axis * tau_R(i);  % τはy軸射影モーメントとして明示
end

% ---- 左脚 ----
I_list_L = {I3, I4, I_wheel_L};
m_idx_L  = [5,6,7];
rot_list_L = rot_L_total;
p_list_L = p_links_L;

syms F_L_out [3 link_num] real
syms F_wL_ground [3 1] real

eqs_L = cell(link_num,2);
tau_L = sym('tau_L', [link_num,1]);

for i = 1:link_num
    m_i = m(m_idx_L(i));
    I_local = I_list_L{i};
    R_i = rot_list_L{i+1};
    I_rot = R_i * I_local * R_i.';

    dp_i = diff(m_i * vLinks_L{i+1}, t_symbol);
    dH_i = diff(I_rot * omegaLinks_L{i+1}, t_symbol);
    Fg_i = m_i * g_vec;

    % 上流の力
    if i == 1
        F_in = F_L;
    else
        F_in = F_L_out(:,i-1);
    end

    % 下流の力
    if i < link_num
        F_out = F_L_out(:,i);
    else
        F_out = -F_wL_ground;
    end

    eqs_L{i,1} = dp_i == Fg_i + F_in - F_out;
    % 関節y軸（ワールド系）
    y_axis = R_i * [0;1;0];
    joint_M = -cross(p_list_L{i}, F_out);
    tau_L(i) = simplify(y_axis.' * joint_M);
    eqs_L{i,2} = dH_i == joint_M + y_axis * tau_L(i);
end

% ---- 結果まとめ ----
NE_eqs = {eq_body, eq_body_ang};
for i=1:link_num
    NE_eqs{end+1} = eqs_R{i,1};
    NE_eqs{end+1} = eqs_R{i,2};
end
for i=1:link_num
    NE_eqs{end+1} = eqs_L{i,1};
    NE_eqs{end+1} = eqs_L{i,2};
end

% ---- 各関節トルクを表示 ----
disp('--- 右脚関節トルク（tau_R）---');
for i=1:link_num
    fprintf('tau_R(%d): ', i); disp(tau_R(i));
end

disp('--- 左脚関節トルク（tau_L）---');
for i=1:link_num
    fprintf('tau_L(%d): ', i); disp(tau_L(i));
end

% ---- 運動方程式一覧 ----
disp('--- ニュートン・オイラー運動方程式一覧 ---');
for k = 1:length(NE_eqs)
    disp(NE_eqs{k})
end