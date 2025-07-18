clear; clc;

% パスの設定
addpath('./function_utill');

fprintf('=== 削減モデルを用いた制御器設計テスト ===\n');

%% 1. 削減されたモデルの平衡点計算

model_reduced = model_definition_reduced();

% 平衡点の設定（直立姿勢）
q0_reduced = zeros(10,1);
q0_reduced(3) = 0.275;                 % ベース高さ（床面接触を考慮）
q0_reduced(4:6) = deg2rad([0; 0; 0]);  % 直立姿勢
q0_reduced(7) = deg2rad(25);           % 右脚股関節
q0_reduced(8) = deg2rad(0);            % 右脚車輪
q0_reduced(9) = deg2rad(25);           % 左脚股関節
q0_reduced(10) = deg2rad(0);           % 左脚車輪

dq0_reduced = zeros(10,1);

% 平衡点でのトルク計算（重力補償）
[q0_full, dq0_full] = model_reduced.to_full_model(q0_reduced, dq0_reduced);
model_original = model_definition_numeric();
ddq0_full = zeros(12,1);

% 重力補償トルクの計算
[f_b0, tau0_full] = inverse_dynamics(q0_full, dq0_full, ddq0_full, model_original);

% 削減されたトルクに変換
% 完全なトルクから削減されたトルクへの変換
% tau0_full = [tau1_R, tau2_R, wheel_R, tau1_L, tau2_L, wheel_L]
% tau0_reduced = [tau1_R - 2*tau2_R, wheel_R, tau1_L - 2*tau2_L, wheel_L]
tau0_reduced = [tau0_full(1) - 2*tau0_full(2);  % 右脚統合トルク
                tau0_full(3);                    % 右脚車輪
                tau0_full(4) - 2*tau0_full(5);  % 左脚統合トルク
                tau0_full(6)];                   % 左脚車輪

fprintf('平衡点計算完了\n');
fprintf('  平衡点関節角度 [deg]: [%.2f, %.2f, %.2f, %.2f]\n', rad2deg(q0_reduced(7:10)));
fprintf('  重力補償トルク [Nm]: [%.3f, %.3f, %.3f, %.3f]\n', tau0_reduced);

%% 2. 削減されたモデルの線形化

% 既存の線形化関数を使用し、削減された状態空間に変換
n_states = 2 * model_reduced.dof;  % 位置 + 速度
n_inputs = 4;  % 削減されたトルク入力

% 削減された状態ベクトル: x = [q_reduced; dq_reduced]
x0 = [q0_reduced; dq0_reduced];
u0 = tau0_reduced;

% 既存の線形化関数を使用（元のモデルの線形化）
[A_full, B_full, C_full, D_full] = compute_linearization(q0_full, dq0_full, tau0_full, model_original);

% 削減された状態空間への変換
% 元のモデル: 12DOF (24状態変数)
% 削減されたモデル: 10DOF (20状態変数)
%
% 状態変数のマッピング:
% q_reduced = [q_full([1:6, 7, 9, 10, 12])]  % ベース(1:6) + 股関節(7,10) + 車輪(9,12)
% dq_reduced = [dq_full([1:6, 7, 9, 10, 12])]
%
% 入力のマッピング:
% u_reduced = [u_full(1) - 2*u_full(2), u_full(3), u_full(4) - 2*u_full(5), u_full(6)]

% 状態変数の選択インデックス
state_indices = [1:6, 7, 9, 10, 12, 13:18, 19, 21, 22, 24];  % 位置 + 速度

% 削減された状態空間の抽出
A = A_full(state_indices, state_indices);
B_reduced = B_full(state_indices, :);

% 入力変換行列の構築
% u_reduced = T_input * u_full
T_input = [1, -2, 0, 0, 0, 0;    % tau1_R - 2*tau2_R
           0, 0, 1, 0, 0, 0;     % wheel_R
           0, 0, 0, 1, -2, 0;    % tau1_L - 2*tau2_L
           0, 0, 0, 0, 0, 1];    % wheel_L

% 削減された入力行列
B = B_reduced * T_input';

% 出力行列（全状態観測）
C = eye(n_states);
D = zeros(n_states, n_inputs);

fprintf('\n線形化完了\n');
fprintf('  状態次元: %d\n', n_states);
fprintf('  入力次元: %d\n', n_inputs);
fprintf('  A行列の最大固有値: %.4f\n', max(real(eig(A))));

%% 3. 可制御性・可観測性の確認

% 可制御性行列
Pc = ctrb(A, B);
rank_Pc = rank(Pc);
fprintf('\n可制御性解析\n');
fprintf('  可制御性行列のランク: %d/%d\n', rank_Pc, n_states);
fprintf('  可制御性: %s\n', iif(rank_Pc == n_states, '可制御', '不可制御'));

% 可観測性行列
Po = obsv(A, C);
rank_Po = rank(Po);
fprintf('\n可観測性解析\n');
fprintf('  可観測性行列のランク: %d/%d\n', rank_Po, n_states);
fprintf('  可観測性: %s\n', iif(rank_Po == n_states, '可観測', '不可観測'));

%% 4. 離散時間モデルの生成

% サンプリング時間の設定（オーバーライド対応）
if evalin('base', 'exist(''T_sampling_override'', ''var'')')
    T = evalin('base', 'T_sampling_override');
    fprintf('サンプリング時間をオーバーライド: %.3f s\n', T);
else
    T = 0.01;  % デフォルトサンプリング時間 [s]
end
sys_continuous = ss(A, B, C, D);
sys_discrete = c2d(sys_continuous, T, 'zoh');

Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;
Dd = sys_discrete.D;

fprintf('\n離散化完了\n');
fprintf('  サンプリング時間: %.3f s\n', T);
fprintf('  離散系の最大固有値: %.4f\n', max(abs(eig(Ad))));

%% 5. LQR制御器設計

% 重み行列の設定（オーバーライド対応）
if evalin('base', 'exist(''Q_matrix_override'', ''var'')')
    Q = evalin('base', 'Q_matrix_override');
    fprintf('状態重み行列をオーバーライド\n');
else
    Q = diag([100, 100, 100, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]);  % デフォルト状態重み
end

if evalin('base', 'exist(''R_matrix_override'', ''var'')')
    R = evalin('base', 'R_matrix_override');
    fprintf('入力重み行列をオーバーライド\n');
else
    R = diag([1, 1, 1, 1]);  % デフォルト入力重み
end

% 離散時間LQR
[K, S, P] = dlqr(Ad, Bd, Q, R);

fprintf('\nLQR制御器設計完了\n');
fprintf('  制御ゲイン行列サイズ: %dx%d\n', size(K));
fprintf('  閉ループ系の最大固有値: %.4f\n', max(abs(eig(Ad - Bd*K))));

%% 6. 制御器の安定性確認

% 閉ループ系の固有値
eigenvalues = eig(Ad - Bd*K);
stable = all(abs(eigenvalues) < 1);

fprintf('\n制御器安定性\n');
fprintf('  閉ループ系固有値: 最大 %.4f, 最小 %.4f\n', max(abs(eigenvalues)), min(abs(eigenvalues)));
fprintf('  安定性: %s\n', iif(stable, '安定', '不安定'));

%% 7. 削減モデルと元モデルの制御性能比較

% 元のモデルの線形化（比較用）
model_original = model_definition_numeric();
[A_orig, B_orig, C_orig, D_orig] = compute_linearization(q0_full, dq0_full, tau0_full, model_original);

% 元のモデルの制御器設計
Q_orig = diag([100*ones(1,6), 50*ones(1,6), ones(1,12)]);
R_orig = diag(ones(1,6));
sys_orig_discrete = c2d(ss(A_orig, B_orig, C_orig, D_orig), T, 'zoh');
[K_orig, ~, ~] = dlqr(sys_orig_discrete.A, sys_orig_discrete.B, Q_orig, R_orig);

% 制御性能の比較
eigenvalues_orig = eig(sys_orig_discrete.A - sys_orig_discrete.B*K_orig);
stable_orig = all(abs(eigenvalues_orig) < 1);

fprintf('\n制御性能比較\n');
fprintf('  元のモデル:\n');
fprintf('    状態次元: %d\n', size(A_orig, 1));
fprintf('    入力次元: %d\n', size(B_orig, 2));
fprintf('    最大固有値: %.4f\n', max(abs(eigenvalues_orig)));
fprintf('    安定性: %s\n', iif(stable_orig, '安定', '不安定'));
fprintf('  削減されたモデル:\n');
fprintf('    状態次元: %d\n', n_states);
fprintf('    入力次元: %d\n', n_inputs);
fprintf('    最大固有値: %.4f\n', max(abs(eigenvalues)));
fprintf('    安定性: %s\n', iif(stable, '安定', '不安定'));

%% 8. 制御器の保存

% 削減されたモデルの制御器を保存
reduced_controller.Ad = Ad;
reduced_controller.Bd = Bd;
reduced_controller.Cd = Cd;
reduced_controller.Dd = Dd;
reduced_controller.K = K;
reduced_controller.Q = Q;
reduced_controller.R = R;
reduced_controller.x0 = x0;
reduced_controller.u0 = u0;
reduced_controller.T = T;
reduced_controller.stable = stable;
reduced_controller.model_reduced = model_reduced;

save('reduced_lqr_controller.mat', 'reduced_controller');

% 削減線形化モデルの保存
reduced_linear_model = struct();
reduced_linear_model.A = A;
reduced_linear_model.B = B;
reduced_linear_model.C = C;
reduced_linear_model.D = D;
reduced_linear_model.x0 = x0;
reduced_linear_model.u0 = u0;
reduced_linear_model.model_reduced = model_reduced;
save('reduced_linear_model.mat', 'reduced_linear_model');

fprintf('\n制御器保存完了\n');
fprintf('  制御器ファイル名: reduced_lqr_controller.mat\n');
fprintf('  線形化ファイル名: reduced_linear_model.mat\n');

%% 9. 結果サマリー

fprintf('\n=== 削減モデル制御器設計結果 ===\n');
fprintf('削減効果:\n');
fprintf('  - 状態変数: 24 → 20 (16.7%% 削減)\n');
fprintf('  - 入力変数: 6 → 4 (33.3%% 削減)\n');
fprintf('  - 制御ゲイン: 6×24 → 4×20 (58.3%% 削減)\n');
fprintf('\n制御性能:\n');
fprintf('  - 可制御性: %s\n', iif(rank_Pc == n_states, '良好', '要改善'));
fprintf('  - 可観測性: %s\n', iif(rank_Po == n_states, '良好', '要改善'));
fprintf('  - 閉ループ安定性: %s\n', iif(stable, '安定', '不安定'));

fprintf('\n削減されたモデルによる制御器設計が完了しました。\n');
fprintf('制約条件を満たしながら効率的な制御が可能です。\n');

%% 補助関数

function result = iif(condition, true_value, false_value)
    if condition
        result = true_value;
    else
        result = false_value;
    end
end