function [theta_full, dtheta_full, ddtheta_full] = compute_joint_constraints(theta_reduced, dtheta_reduced, ddtheta_reduced)
% 関節制約を考慮した完全な関節角度・速度・加速度の計算
% 制約: theta2 = -2*theta1 (各脚において)
%
% 入力:
%   theta_reduced: 削減された関節角度 [theta1_R, wheel_R, theta1_L, wheel_L] (4x1)
%   dtheta_reduced: 削減された関節角速度 (4x1)
%   ddtheta_reduced: 削減された関節角加速度 (4x1)
%
% 出力:
%   theta_full: 完全な関節角度 [theta1_R, theta2_R, wheel_R, theta1_L, theta2_L, wheel_L] (6x1)
%   dtheta_full: 完全な関節角速度 (6x1)
%   ddtheta_full: 完全な関節角加速度 (6x1)

% 関節角度の展開
theta1_R = theta_reduced(1);
wheel_R = theta_reduced(2);
theta1_L = theta_reduced(3);
wheel_L = theta_reduced(4);

% 制約関係: theta2 = -2*theta1
theta2_R = -2 * theta1_R;
theta2_L = -2 * theta1_L;

% 完全な関節角度
theta_full = [theta1_R; theta2_R; wheel_R; theta1_L; theta2_L; wheel_L];

% 関節角速度の展開（制約の時間微分）
dtheta1_R = dtheta_reduced(1);
dwheel_R = dtheta_reduced(2);
dtheta1_L = dtheta_reduced(3);
dwheel_L = dtheta_reduced(4);

% 制約関係の微分: dtheta2 = -2*dtheta1
dtheta2_R = -2 * dtheta1_R;
dtheta2_L = -2 * dtheta1_L;

% 完全な関節角速度
dtheta_full = [dtheta1_R; dtheta2_R; dwheel_R; dtheta1_L; dtheta2_L; dwheel_L];

% 関節角加速度の展開（制約の二階微分）
ddtheta1_R = ddtheta_reduced(1);
ddwheel_R = ddtheta_reduced(2);
ddtheta1_L = ddtheta_reduced(3);
ddwheel_L = ddtheta_reduced(4);

% 制約関係の二階微分: ddtheta2 = -2*ddtheta1
ddtheta2_R = -2 * ddtheta1_R;
ddtheta2_L = -2 * ddtheta1_L;

% 完全な関節角加速度
ddtheta_full = [ddtheta1_R; ddtheta2_R; ddwheel_R; ddtheta1_L; ddtheta2_L; ddwheel_L];

end