clear; clc;

model = model_definition_numeric();
q = zeros(12,1);
dq = zeros(12,1);
tau = zeros(6,1);

q(4:6) = deg2rad([0; 0; 80]);
q(7:9) = deg2rad([0; 0; 0]);
q(10:12) = deg2rad([0; 0; 0]);
q(7) = deg2rad(38);
q(8) = -2*q(7);
q(10) = deg2rad(38);
q(11) = -2*q(10);

% v_b は y軸方向へは直接動けない（ノンホロノミック）
v_b = [0.1; 0; 0];
w_b = [0; 0.1; 0];

kin = compute_kinematics(q, model);
% ロボットの3D可視化
visualize_robot(kin, model);

[vLinks_R, omegaLinks_R, vLinks_L, omegaLinks_L] = forward_velocity(v_b, w_b, q, dq, model);

dq(1:3) = v_b;
dq(4:6) = w_b;

ddq0 = zeros(12,1);
[f_b0, tau0] = inverse_dynamics(q, dq, ddq0, model);
disp('--- inverse_dynamics (C + G) ---');
disp('f_b0 (base force & moment):');
disp(f_b0);
disp('tau0 (joint torques):');
disp(tau0);

tau = [5; -5; 0.2; -0.2; 0.1; -0.1];
ddq = forward_dynamics(q, dq, tau, model);
disp('--- forward_dynamics (ddq) ---');
disp(ddq);
