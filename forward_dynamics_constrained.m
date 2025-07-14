function [ddq, lambda] = forward_dynamics_constrained(q, dq, tau, model)
% 拘束条件下での順動力学計算
% 床面接触拘束を考慮した運動方程式を解く
%
% 拘束条件付き運動方程式：
% M*ddq + J_phi'*lambda = Q
% J_phi*ddq = -J_phi_dot*dq
%
% 入力：
%   q    : 一般化座標 [12x1]
%   dq   : 一般化速度 [12x1]  
%   tau  : 関節トルク [6x1]
%   model: ロボットモデル
%
% 出力：
%   ddq   : 一般化加速度 [12x1]
%   lambda: 拘束力（床反力） [2x1]

% 質量行列の計算（従来の方法）
n = length(q);
M = zeros(n, n);

for i = 1:n
    ddq_unit = zeros(n, 1);
    ddq_unit(i) = 1.0;
    [f_b, tau_] = inverse_dynamics_unit(q, dq, ddq_unit, model);
    M(:,i) = [f_b; tau_];
end

% 重力・コリオリ項の計算
ddq_zero = zeros(n, 1);
[f_b0, tau0] = inverse_dynamics_unit(q, dq, ddq_zero, model);
C_G = [f_b0; tau0];

% 拘束条件の計算
[phi, J_phi, J_phi_dot] = compute_constraints(q, dq, model);

% 一般化力の設定
Q = [zeros(6,1); tau] - C_G;  % 外力なし、関節トルクのみ

% 拘束条件付き運動方程式の構築
% [M   J_phi'] [ddq   ]   [Q                ]
% [J_phi  0  ] [lambda] = [-J_phi_dot*dq    ]

nc = size(J_phi, 1);  % 拘束条件の数
A = [M, J_phi'; J_phi, zeros(nc, nc)];
b = [Q; -J_phi_dot*dq];

% 数値的安定性のための正則化
lambda_reg = 1e-8;
A(1:n, 1:n) = A(1:n, 1:n) + lambda_reg * eye(n);

% 連立方程式の解法
solution = A \ b;

ddq = solution(1:n);
lambda = solution(n+1:end);

% 拘束条件の確認（デバッグ用）
phi_ddot = J_phi * ddq + J_phi_dot * dq;
if max(abs(phi_ddot)) > 1e-6
    warning('拘束条件が満たされていません: max(|phi_ddot|) = %e', max(abs(phi_ddot)));
end

end