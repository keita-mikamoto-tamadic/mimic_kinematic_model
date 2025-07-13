function kin = compute_kinematics(q, model)
x_b = q(1:3);
eul_b = q(4:6);
theta1 = q(7);
theta2 = q(8);
theta_wR = q(9);
theta3 = q(10);
theta4 = q(11);
theta_wL = q(12);

R_b = model.r_base(eul_b(1), eul_b(2), eul_b(3));
p_b = x_b;

Ry = model.ry;
rot_R = {Ry(theta1); Ry(theta2); Ry(theta_wR)};
rot_L = {Ry(theta3); Ry(theta4); Ry(theta_wL)};

rot_R_total = cell(model.link_num+1, 1);
rot_L_total = cell(model.link_num+1, 1);
rot_R_total{1} = R_b;
rot_L_total{1} = R_b;

for i = 1:model.link_num
    rot_R_total{i+1} = rot_R{i} * rot_R_total{i};
    rot_L_total{i+1} = rot_L{i} * rot_L_total{i};
end

p_links_R = cell(model.link_num, 1);
p_links_L = cell(model.link_num, 1);
c_links_R = cell(model.link_num, 1);
c_links_L = cell(model.link_num, 1);

for i = 1:model.link_num
    if i == 1
        % 最初のリンクはベースから
        p_links_R{i} = p_b + rot_R_total{i} * model.p_links_R{i};
        p_links_L{i} = p_b + rot_L_total{i} * model.p_links_L{i};
    else
        p_links_R{i} = p_links_R{i-1} + rot_R_total{i} * model.p_links_R{i};
        p_links_L{i} = p_links_L{i-1} + rot_L_total{i} * model.p_links_L{i};
    end
    c_links_R{i} = p_links_R{i} + rot_R_total{i+1} * model.c_links_R{i};
    c_links_L{i} = p_links_L{i} + rot_L_total{i+1} * model.c_links_L{i};
end

kin.p_b = p_b;
kin.R_b = R_b;
kin.p_links_R = p_links_R;
kin.p_links_L = p_links_L;
kin.c_links_R = c_links_R;
kin.c_links_L = c_links_L;
kin.rot_R_total = rot_R_total;
kin.rot_L_total = rot_L_total;
end
