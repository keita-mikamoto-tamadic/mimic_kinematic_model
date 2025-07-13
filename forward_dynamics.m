function ddq = forward_dynamics(q, dq, tau, model)
n = length(q);
M = zeros(n, n);
ddq0 = zeros(n,1);

[f_b0, tau0] = inverse_dynamics(q, dq, ddq0, model);
b = [f_b0; tau0];

for i = 1:n
    ddq = zeros(n,1);
    ddq(i) = 1.0;
    [f_b, tau_] = inverse_dynamics(q, dq, ddq, model);
    M(:,i) = [f_b; tau_] - b;
end
disp('rank(M):');
disp(rank(M));
disp('cond(M):');
disp(cond(M));
disp('M =');
disp(M);


% 数値的な安定性のために小さな正則化項を追加
lambda = 1e-8;
M_reg = M + lambda * eye(size(M));

u = [zeros(6,1); tau];
ddq = M_reg \ (u - b);
end
