function [vLinks_R, omegaLinks_R, vLinks_L, omegaLinks_L] = forward_velocity(v_b, w_b, q, dq, model)
kin = compute_kinematics(q, model);
link_num = model.link_num;

vLinks_R = cell(link_num+1, 1);
omegaLinks_R = cell(link_num+1, 1);
vLinks_L = cell(link_num+1, 1);
omegaLinks_L = cell(link_num+1, 1);

vLinks_R{1} = v_b;
omegaLinks_R{1} = w_b;
vLinks_L{1} = v_b;
omegaLinks_L{1} = w_b;

% dq [7:12] : 股, 膝, ホイール
dq_R = dq(7:9);
dq_L = dq(10:12);

for i = 1:link_num
    R_R = kin.rot_R_total{i};
    p_R = model.p_links_R{i};
    omegaLinks_R{i+1} = omegaLinks_R{i} + R_R(:,2) * dq_R(i);
    vLinks_R{i+1} = vLinks_R{i} + cross(omegaLinks_R{i}, R_R * p_R);

    R_L = kin.rot_L_total{i};
    p_L = model.p_links_L{i};
    omegaLinks_L{i+1} = omegaLinks_L{i} + R_L(:,2) * dq_L(i);
    vLinks_L{i+1} = vLinks_L{i} + cross(omegaLinks_L{i}, R_L * p_L);
end
end
