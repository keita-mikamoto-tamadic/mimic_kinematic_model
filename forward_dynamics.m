function ddq = forward_dynamics(q, dq, tau, model)
% 単位ベクトル法による運動方程式の解法
% M*ddq = f の質量行列Mを解析的に構築

n = length(q);
M = zeros(n, n);

% 各自由度に単位加速度を与えて質量行列を構築
for i = 1:n
    ddq_unit = zeros(n, 1);
    ddq_unit(i) = 1.0;  % 第i自由度に単位加速度
    
    % 微分運動学による速度・加速度の伝播
    [f_b, tau_] = inverse_dynamics_unit(q, dq, ddq_unit, model);
    M(:,i) = [f_b; tau_];
end

% 重力・コリオリ項の計算（全加速度0の場合）
ddq_zero = zeros(n, 1);
[f_b0, tau0] = inverse_dynamics_unit(q, dq, ddq_zero, model);
b = [f_b0; tau0];

% 数値的な安定性のために小さな正則化項を追加
lambda = 1e-8;
M_reg = M + lambda * eye(size(M));

% 運動方程式の解法: M*ddq = u - b
u = [zeros(6,1); tau];  % 外力はゼロ、関節トルクのみ
ddq = M_reg \ (u - b);
end
