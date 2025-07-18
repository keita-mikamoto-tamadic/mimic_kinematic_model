function run_system()
% 脚車輪型倒立振子削減モデル制御システム - 統合実行ファイル

addpath('./function_utill');

fprintf('=== 脚車輪型倒立振子削減モデル制御システム ===\n\n');

%% 1. 削減モデルテスト実行
fprintf('[1/3] 削減モデルテスト実行中...\n');
run_reduced_model_test;

%% 2. 制御設計パラメータ設定
fprintf('\n[2/5] 制御設計パラメータ設定中...\n');

% サンプリング周期設定
T_sampling = 0.01;  % 100Hz制御サンプリング [s]
fprintf('制御サンプリング周期: T = %.3f [s] (%.0f Hz)\n', T_sampling, 1/T_sampling);

% LQR重み行列設定
% 状態重み行列 Q (20x20)
Q_weights = [100, 100, 100, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
Q_matrix = diag(Q_weights);

% 制御入力重み行列 R (4x4)
R_weights = [1, 1, 1, 1];  % 制御入力重み (均等)
R_matrix = diag(R_weights);

fprintf('LQR重み設定:\n');
fprintf('  状態重み Q: ベース位置=%.0f, ベース姿勢=%.0f, 関節=%.0f\n', ...
    Q_weights(1), Q_weights(4), Q_weights(7));
fprintf('  入力重み R: 制御入力=%.0f (均等)\n', R_weights(1));

% パラメータをワークスペースに保存（run_reduced_control_testで使用）
assignin('base', 'T_sampling_override', T_sampling);
assignin('base', 'Q_matrix_override', Q_matrix);
assignin('base', 'R_matrix_override', R_matrix);

%% 3. 削減モデル制御器設計
fprintf('\n[3/5] 削減モデル制御器設計中...\n');
run_reduced_control_test;

%% 4. 100Hz制御サンプリング用離散化
fprintf('\n[4/5] 100Hz制御サンプリング用離散化中...\n');

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

%% 5. フィードバックゲイン行列表示と安定性確認
fprintf('\n[5/5] フィードバックゲイン行列表示と安定性確認中...\n');

% 制御器データの読み込み
load('reduced_lqr_controller.mat');

fprintf('\n=== フィードバックゲイン行列 ===\n');
fprintf('K = \n');
K = reduced_controller.K;
for i = 1:4
    fprintf('行%d: ', i);
    for j = 1:20
        fprintf('%8.4f ', K(i,j));
    end
    fprintf('\n');
end
fprintf('サイズ: %d × %d\n', size(reduced_controller.K));
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
fprintf('- 100Hz制御サンプリング (T=%.3fs) で離散化済み\n', T_sampling);
if all_stable
    fprintf('- LQR制御器は安定 (全極 |λ|<1)\n');
else
    fprintf('- LQR制御器は不安定 (要調整)\n');
end

end