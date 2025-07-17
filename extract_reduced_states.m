function [q_reduced, dq_reduced] = extract_reduced_states(q_full, dq_full)
% 完全な状態から削減された状態を抽出
% 制約: theta2 = 2*theta1 を考慮してtheta2を除去
%
% 入力:
%   q_full: 完全な一般化座標 [base(6), theta1_R, theta2_R, wheel_R, theta1_L, theta2_L, wheel_L] (12x1)
%   dq_full: 完全な一般化速度 (12x1)
%
% 出力:
%   q_reduced: 削減された一般化座標 [base(6), theta1_R, wheel_R, theta1_L, wheel_L] (10x1)
%   dq_reduced: 削減された一般化速度 (10x1)

% ベース部分はそのまま
base_q = q_full(1:6);
base_dq = dq_full(1:6);

% 関節角度の抽出（theta2を除去）
theta1_R = q_full(7);
theta2_R = q_full(8);  % 制約チェック用
wheel_R = q_full(9);
theta1_L = q_full(10);
theta2_L = q_full(11);  % 制約チェック用
wheel_L = q_full(12);

% 制約条件の確認（許容誤差内での制約満足度チェック）
constraint_error_R = abs(theta2_R - 2*theta1_R);
constraint_error_L = abs(theta2_L - 2*theta1_L);
tolerance = 1e-6;

if constraint_error_R > tolerance
    warning('右脚の制約条件が満たされていません: |theta2_R - 2*theta1_R| = %.6f', constraint_error_R);
end

if constraint_error_L > tolerance
    warning('左脚の制約条件が満たされていません: |theta2_L - 2*theta1_L| = %.6f', constraint_error_L);
end

% 削減された関節角度
joints_q_reduced = [theta1_R; wheel_R; theta1_L; wheel_L];

% 関節角速度の抽出（theta2の角速度を除去）
dtheta1_R = dq_full(7);
dtheta2_R = dq_full(8);  % 制約チェック用
dwheel_R = dq_full(9);
dtheta1_L = dq_full(10);
dtheta2_L = dq_full(11);  % 制約チェック用
dwheel_L = dq_full(12);

% 制約条件の微分の確認
dconstraint_error_R = abs(dtheta2_R - 2*dtheta1_R);
dconstraint_error_L = abs(dtheta2_L - 2*dtheta1_L);

if dconstraint_error_R > tolerance
    warning('右脚の速度制約条件が満たされていません: |dtheta2_R - 2*dtheta1_R| = %.6f', dconstraint_error_R);
end

if dconstraint_error_L > tolerance
    warning('左脚の速度制約条件が満たされていません: |dtheta2_L - 2*dtheta1_L| = %.6f', dconstraint_error_L);
end

% 削減された関節角速度
joints_dq_reduced = [dtheta1_R; dwheel_R; dtheta1_L; dwheel_L];

% 結果の組み立て
q_reduced = [base_q; joints_q_reduced];
dq_reduced = [base_dq; joints_dq_reduced];

end