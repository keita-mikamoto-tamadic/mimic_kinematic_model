%% スキュー行列関数
vee = @(S) [S(3,2); S(1,3); S(2,1)];
skew = @(v) [  0    -v(3)  v(2);
              v(3)   0    -v(1);
             -v(2)  v(1)   0  ];

%% ボディ速度ベクトル
v_b = diff(p_b, t_symbol); % p_bは[ x_b; y_b; z_b ] (t_symbol依存)

%% 右脚 速度計算
vLinks_R = cell(link_num + 1, 1);      % 原点速度
omegaLinks_R = cell(link_num + 1, 1);  % 角速度
vc_links_R = cell(link_num, 1);        % 重心速度

vLinks_R{1} = v_b;
R = rot_R_total{1};
dR = diff(R, t_symbol);  
omega_skew = dR * R.';
omegaLinks_R{1} = vee(omega_skew);

v_cg_body = v_b + omega_skew * c_b;   % ボディ重心速度

for i = 1:link_num
    R = rot_R_total{i+1};
    dR = diff(R, t_symbol);
    omega_skew = dR * R.';
    omegaLinks_R{i+1} = vee(omega_skew);

    rel_pos = rot_R_total{i+1} * p_links_R{i};
    vLinks_R{i+1} = vLinks_R{i} + skew(omegaLinks_R{i}) * rel_pos;

    rel_cog = rot_R_total{i+1} * c_links_R{i};
    vc_links_R{i} = vLinks_R{i+1} + skew(omegaLinks_R{i+1}) * rel_cog;
end

%% 左脚 速度計算（同じロジック）
vLinks_L = cell(link_num + 1, 1);
omegaLinks_L = cell(link_num + 1, 1);
vc_links_L = cell(link_num, 1);

vLinks_L{1} = v_b;
R = rot_L_total{1};
dR = diff(R, t_symbol);
omega_skew = dR * R.';
omegaLinks_L{1} = vee(omega_skew);

for i = 1:link_num
    R = rot_L_total{i+1};
    dR = diff(R, t_symbol);
    omega_skew = dR * R.';
    omegaLinks_L{i+1} = vee(omega_skew);
    disp(size(omegaLinks_L{i}))

    rel_pos = rot_L_total{i+1} * p_links_L{i};
    vLinks_L{i+1} = vLinks_L{i} + skew(omegaLinks_L{i}) * rel_pos;

    rel_cog = rot_L_total{i+1} * c_links_L{i};
    vc_links_L{i} = vLinks_L{i+1} + skew(omegaLinks_L{i+1}) * rel_cog;
end
