function [A, B, C, D] = compute_linearization(q0, dq0, tau0, model)
% テイラー1次展開による線形化
% 平衡点周りでの状態方程式 dx/dt = Ax + Bu を導出
%
% 入力:
%   q0   : 平衡点での一般化座標 [12x1]
%   dq0  : 平衡点での一般化速度 [12x1]
%   tau0 : 平衡点での関節トルク [6x1]
%   model: ロボットモデル
%
% 出力:
%   A : 状態行列 [24x24]
%   B : 入力行列 [24x6]
%   C : 出力行列 [12x24]
%   D : 直達行列 [12x6]

fprintf('=== テイラー1次展開による線形化 ===\n');

% 状態変数の定義
% x = [q - q0; dq - dq0] (24x1)
% u = tau - tau0 (6x1)

n_q = length(q0);      % 12 (一般化座標の数)
n_u = length(tau0);    % 6  (関節トルクの数)
n_x = 2 * n_q;         % 24 (状態変数の数)

% 数値微分のステップ幅
delta = 1e-6;

% A行列の計算
A = zeros(n_x, n_x);

fprintf('A行列の計算中...\n');

for i = 1:n_x
    % 状態変数の摂動
    x_plus = zeros(n_x, 1);
    x_plus(i) = delta;
    
    % 摂動した状態での座標と速度
    if i <= n_q
        q_plus = q0 + x_plus(1:n_q);
        dq_plus = dq0 + x_plus(n_q+1:end);
    else
        q_plus = q0 + x_plus(1:n_q);
        dq_plus = dq0 + x_plus(n_q+1:end);
    end
    
    % 摂動した状態での動力学
    try
        [ddq_plus, ~] = forward_dynamics_constrained(q_plus, dq_plus, tau0, model);
        dx_plus = [dq_plus; ddq_plus];
        
        % 平衡点での動力学
        [ddq0, ~] = forward_dynamics_constrained(q0, dq0, tau0, model);
        dx0 = [dq0; ddq0];
        
        % 偏微分の計算
        A(:, i) = (dx_plus - dx0) / delta;
        
    catch ME
        warning('A行列の計算でエラーが発生しました (列 %d): %s', i, ME.message);
        % エラーが発生した場合は0で埋める
        A(:, i) = zeros(n_x, 1);
    end
    
    if mod(i, 6) == 0
        fprintf('  進捗: %d/%d\n', i, n_x);
    end
end

% B行列の計算
B = zeros(n_x, n_u);

fprintf('B行列の計算中...\n');

for i = 1:n_u
    % 入力の摂動
    tau_plus = tau0;
    tau_plus(i) = tau_plus(i) + delta;
    
    % 摂動した入力での動力学
    try
        [ddq_plus, ~] = forward_dynamics_constrained(q0, dq0, tau_plus, model);
        dx_plus = [dq0; ddq_plus];
        
        % 平衡点での動力学
        [ddq0, ~] = forward_dynamics_constrained(q0, dq0, tau0, model);
        dx0 = [dq0; ddq0];
        
        % 偏微分の計算
        B(:, i) = (dx_plus - dx0) / delta;
        
    catch ME
        warning('B行列の計算でエラーが発生しました (列 %d): %s', i, ME.message);
        % エラーが発生した場合は0で埋める
        B(:, i) = zeros(n_x, 1);
    end
end

% C行列とD行列（出力方程式）
% 出力を位置と姿勢のみとする場合
C = [eye(n_q), zeros(n_q, n_q)];  % y = q
D = zeros(n_q, n_u);

% 線形化モデルの特性を表示
fprintf('\n=== 線形化モデルの特性 ===\n');
fprintf('状態変数の数: %d (位置: %d, 速度: %d)\n', n_x, n_q, n_q);
fprintf('入力変数の数: %d\n', n_u);
fprintf('出力変数の数: %d\n', size(C, 1));

% A行列の固有値
eigenvalues = eig(A);
fprintf('A行列の固有値:\n');
for i = 1:length(eigenvalues)
    if imag(eigenvalues(i)) == 0
        fprintf('  λ%d = %7.4f\n', i, real(eigenvalues(i)));
    else
        fprintf('  λ%d = %7.4f + %7.4fi\n', i, real(eigenvalues(i)), imag(eigenvalues(i)));
    end
end

% 安定性の判定
stable_poles = all(real(eigenvalues) < 0);
if stable_poles
    fprintf('システムの安定性: 安定\n');
else
    fprintf('システムの安定性: 不安定\n');
end

% 可制御性の確認
try
    Co = ctrb(A, B);
    rank_Co = rank(Co);
    if rank_Co == n_x
        fprintf('可制御性: 完全可制御 (rank = %d/%d)\n', rank_Co, n_x);
    else
        fprintf('可制御性: 部分可制御 (rank = %d/%d)\n', rank_Co, n_x);
    end
catch
    fprintf('可制御性の確認でエラーが発生しました\n');
end

% 可観測性の確認
try
    Ob = obsv(A, C);
    rank_Ob = rank(Ob);
    if rank_Ob == n_x
        fprintf('可観測性: 完全可観測 (rank = %d/%d)\n', rank_Ob, n_x);
    else
        fprintf('可観測性: 部分可観測 (rank = %d/%d)\n', rank_Ob, n_x);
    end
catch
    fprintf('可観測性の確認でエラーが発生しました\n');
end

end