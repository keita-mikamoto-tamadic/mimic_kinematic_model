function run_system()
% 脚車輪型倒立振子削減モデル制御システム - 統合実行ファイル

addpath('./function_utill');

fprintf('=== 脚車輪型倒立振子削減モデル制御システム ===\n\n');

%% 1. 削減モデルテスト実行
fprintf('[1/6] 削減モデルテスト実行中...\n');
run_reduced_model_test;

%% 2. 制御設計パラメータ設定
fprintf('\n[2/6] 制御設計パラメータ設定中...\n');

% サンプリング周期設定
T_sampling = 0.01;  % 100Hz制御サンプリング [s]
fprintf('制御サンプリング周期: T = %.3f [s] (%.0f Hz)\n', T_sampling, 1/T_sampling);

% 重み調整方針の選択
weight_tuning_method = 'basic';  % 'basic', 'attitude', 'high', 'iterative', 'none'
fprintf('重み調整方針: %s\n', weight_tuning_method);

% 重み調整方針をワークスペースに保存
assignin('base', 'weight_tuning_method', weight_tuning_method);

% デフォルト重み設定 (重み調整なしの場合)
Q_weights_default = [100, 100, 100, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
R_weights_default = [1, 1, 1, 1];  % 制御入力重み (均等)

% パラメータをワークスペースに保存（run_reduced_control_testで使用）
assignin('base', 'T_sampling_override', T_sampling);
assignin('base', 'Q_matrix_override', diag(Q_weights_default));
assignin('base', 'R_matrix_override', diag(R_weights_default));

%% 3. 削減モデル制御器設計
fprintf('\n[3/6] 削減モデル制御器設計中...\n');
run_reduced_control_test;

%% 4. 100Hz制御サンプリング用離散化
fprintf('\n[4/6] 100Hz制御サンプリング用離散化中...\n');

% 削減モデルの線形化データを読み込み
load('reduced_linear_model.mat');
A = reduced_linear_model.A;
B = reduced_linear_model.B;
C = reduced_linear_model.C;
D = reduced_linear_model.D;

% 100Hz制御サンプリング（T=0.01s）で離散化
T_sampling = 0.01;  % 10ms = 100Hz
fprintf('制御サンプリング時間: T = %.3f [s] (%.0f Hz)\n', T_sampling, 1/T_sampling);
[Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T_sampling, 'zoh');

% 離散化モデルを保存
discrete_reduced_model = struct();
discrete_reduced_model.Ad = Ad;
discrete_reduced_model.Bd = Bd;
discrete_reduced_model.Cd = Cd;
discrete_reduced_model.Dd = Dd;
discrete_reduced_model.T = T_sampling;
discrete_reduced_model.method = 'zoh';
save('discrete_reduced_model.mat', 'discrete_reduced_model');
fprintf('離散化モデルを discrete_reduced_model.mat に保存しました\n');

%% 5. LQR重み調整
fprintf('\n[5/6] LQR重み調整中...\n');

% モデル情報の構造化
model_info = struct();
model_info.n_states = 20;
model_info.n_inputs = 4;
model_info.sampling_time = T_sampling;

% 重み調整方針による分岐
weight_tuning_method = evalin('base', 'weight_tuning_method');
switch weight_tuning_method
    case 'basic'
        [Q_tuned, R_tuned, K_tuned, stability_info] = tune_basic_weights(Ad, Bd, model_info);
    case 'attitude'
        [Q_tuned, R_tuned, K_tuned, stability_info] = tune_attitude_weights(Ad, Bd, model_info);
    case 'high'
        [Q_tuned, R_tuned, K_tuned, stability_info] = tune_high_weights(Ad, Bd, model_info);
    case 'iterative'
        [Q_tuned, R_tuned, K_tuned, stability_info] = tune_iterative_weights(Ad, Bd, model_info);
    case 'none'
        fprintf('重み調整をスキップします（デフォルト重み使用）\n');
        Q_tuned = diag(Q_weights_default);
        R_tuned = diag(R_weights_default);
        [K_tuned, ~, ~] = dlqr(Ad, Bd, Q_tuned, R_tuned);
        
        % 基本的な安定性情報
        Ad_cl = Ad - Bd * K_tuned;
        eigenvalues_cl = eig(Ad_cl);
        stability_info = struct();
        stability_info.is_stable = all(abs(eigenvalues_cl) < 1);
        stability_info.max_eigenvalue = max(abs(eigenvalues_cl));
        stability_info.stability_margin = 1 - max(abs(eigenvalues_cl));
        stability_info.control_norm = norm(K_tuned);
        stability_info.eigenvalues = eigenvalues_cl;
        stability_info.method = 'none';
    otherwise
        error('未知の重み調整方針: %s', weight_tuning_method);
end

% 調整された重みをオーバーライド
if ~strcmp(weight_tuning_method, 'none')
    assignin('base', 'Q_matrix_override', Q_tuned);
    assignin('base', 'R_matrix_override', R_tuned);
    
    % 制御器の再設計
    fprintf('\n調整された重みで制御器を再設計中...\n');
    run_reduced_control_test;
end

%% 6. フィードバックゲイン行列表示と安定性確認
fprintf('\n[6/6] フィードバックゲイン行列表示と安定性確認中...\n');

% 制御器データの読み込み
load('reduced_lqr_controller.mat');

fprintf('\n=== システム行列 A (離散化) ===\n');
fprintf('Ad = \n');
fprintf('サイズ: %d × %d\n', size(Ad));
for i = 1:size(Ad,1)
    fprintf('行%2d: ', i);
    for j = 1:size(Ad,2)
        fprintf('%8.4f ', Ad(i,j));
    end
    fprintf('\n');
end

fprintf('\n=== 入力行列 B (離散化) ===\n');
fprintf('Bd = \n');
fprintf('サイズ: %d × %d\n', size(Bd));
for i = 1:size(Bd,1)
    fprintf('行%2d: ', i);
    for j = 1:size(Bd,2)
        fprintf('%8.4f ', Bd(i,j));
    end
    fprintf('\n');
end

fprintf('\n=== 最終重み行列 Q ===\n');
fprintf('Q = \n');
Q_final = reduced_controller.Q;
fprintf('サイズ: %d × %d\n', size(Q_final));
for i = 1:size(Q_final,1)
    fprintf('行%2d: ', i);
    for j = 1:size(Q_final,2)
        fprintf('%8.4f ', Q_final(i,j));
    end
    fprintf('\n');
end

fprintf('\n=== 最終重み行列 R ===\n');
fprintf('R = \n');
R_final = reduced_controller.R;
fprintf('サイズ: %d × %d\n', size(R_final));
for i = 1:size(R_final,1)
    fprintf('行%2d: ', i);
    for j = 1:size(R_final,2)
        fprintf('%8.4f ', R_final(i,j));
    end
    fprintf('\n');
end

fprintf('\n=== フィードバックゲイン行列 K ===\n');
fprintf('K = \n');
K = reduced_controller.K;
fprintf('サイズ: %d × %d\n', size(K));
for i = 1:size(K,1)
    fprintf('行%2d: ', i);
    for j = 1:size(K,2)
        fprintf('%8.4f ', K(i,j));
    end
    fprintf('\n');
end
fprintf('制御則: u = -K * (x - x_eq) + u_eq\n');

%% 5. LQR制御器の極計算と安定性確認
fprintf('\n=== LQR制御器の安定性確認 ===\n');

% 閉ループ系の計算
Ad_cl = Ad - Bd * K;
eigenvalues_cl = eig(Ad_cl);

% 極の表示
fprintf('離散時間閉ループ系の極 (固有値):\n');
for i = 1:length(eigenvalues_cl)
    magnitude = abs(eigenvalues_cl(i));
    if imag(eigenvalues_cl(i)) == 0
        fprintf('  λ%2d = %8.5f (|λ| = %7.4f)', i, real(eigenvalues_cl(i)), magnitude);
    else
        fprintf('  λ%2d = %8.5f + %8.5fi (|λ| = %7.4f)', i, real(eigenvalues_cl(i)), imag(eigenvalues_cl(i)), magnitude);
    end
    
    % 安定性判定
    if magnitude < 1
        fprintf(' [安定]\n');
    else
        fprintf(' [不安定]\n');
    end
end

% 全体の安定性判定
all_stable = all(abs(eigenvalues_cl) < 1);
max_magnitude = max(abs(eigenvalues_cl));

fprintf('\n安定性判定結果:\n');
if all_stable
    fprintf('  ✓ すべての極の絶対値が1未満: 離散時間システムは安定\n');
    fprintf('  最大極絶対値: %.6f < 1.000000\n', max_magnitude);
else
    fprintf('  ✗ 不安定な極が存在: 離散時間システムは不安定\n');
    fprintf('  最大極絶対値: %.6f >= 1.000000\n', max_magnitude);
    warning('制御器が不安定です。LQR重み行列の再調整が必要です。');
end

fprintf('\n=== システム実行完了 ===\n');
fprintf('制御システムの準備が完了しました。\n');
fprintf('- 100Hz制御サンプリング (T=%.3fs) で離散化済み\n', 0.01);
% 重み調整方針の取得
weight_tuning_method = evalin('base', 'weight_tuning_method');
fprintf('- 重み調整方針: %s\n', weight_tuning_method);

if all_stable
    fprintf('- LQR制御器は安定 (全極 |λ|<1)\n');
else
    fprintf('- LQR制御器は不安定 (要調整)\n');
end

% 重み調整結果の表示
if ~strcmp(weight_tuning_method, 'none')
    fprintf('\n=== 重み調整結果 ===\n');
    fprintf('調整方針: %s\n', weight_tuning_method);
    fprintf('調整済み制御器が適用されました\n');
end

end