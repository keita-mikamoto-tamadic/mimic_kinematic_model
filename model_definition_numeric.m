function model = model_definition_numeric()
param_definition;

model.g_vec = [0; 0; -9.81];
model.link_num = 3;
model.wheel_radius = wheel_radius;
model.m_b = m(1);
model.m_link_R = m(2:4);
model.m_link_L = m(5:7);
model.Ib = Ib;
model.I_list_R = {I1, I2, I_wheel_R};
model.I_list_L = {I3, I4, I_wheel_L};

model.c_b = [cogx_b; cogy_b; cogz_b];
model.c_links_R = { [l1_cogx; l1_cogy; l1_cogz]; [l2_cogx; l2_cogy; l2_cogz]; [wR_cogx; wR_cogy; wR_cogz] };
model.c_links_L = { [l3_cogx; l3_cogy; l3_cogz]; [l4_cogx; l4_cogy; l4_cogz]; [wL_cogx; wL_cogy; wL_cogz] };
model.p_links_R = { [l1_x; l1_y; l1_z]; [l2_x; l2_y; l2_z]; [wR_x; wR_y; wR_z] };
model.p_links_L = { [l3_x; l3_y; l3_z]; [l4_x; l4_y; l4_z]; [wL_x; wL_y; wL_z] };

model.ry = @(theta) rodrigues([0; 1; 0], theta);
model.r_base = @(phi, theta, psi) rodrigues([1; 0; 0], phi) * rodrigues([0; 1; 0], theta) * rodrigues([0; 0; 1], psi);
end
