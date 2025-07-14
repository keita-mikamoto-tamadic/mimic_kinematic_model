clear; clc;

% パスの設定
addpath('./function_utill');

fprintf('=== 脚車輪型倒立振子の離散化テスト ===\n\n');

%% 1. 線形化モデルの読み込み
if exist('linear_model.mat', 'file')
    load('linear_model.mat');
    fprintf('線形化モデルを読み込みました\n');
else
    fprintf('線形化モデルが見つかりません。先に run_linearization_test.m を実行してください\n');
    return;
end

A = linear_model.A;
B = linear_model.B;
C = linear_model.C;
D = linear_model.D;

%% 2. 離散化パラメータの設定
% サンプリング時間の設定（後で変更可能）
sampling_config = struct();
sampling_config.T_default = 0.01;      % デフォルト: 10ms (100Hz)
sampling_config.T_fast = 0.005;        % 高速: 5ms (200Hz)
sampling_config.T_slow = 0.05;         % 低速: 50ms (20Hz)
sampling_config.method = 'zoh';        % 離散化手法

fprintf('=== サンプリング時間の設定 ===\n');
fprintf('デフォルト: T = %g [s] (%g Hz)\n', sampling_config.T_default, 1/sampling_config.T_default);
fprintf('高速サンプリング: T = %g [s] (%g Hz)\n', sampling_config.T_fast, 1/sampling_config.T_fast);
fprintf('低速サンプリング: T = %g [s] (%g Hz)\n', sampling_config.T_slow, 1/sampling_config.T_slow);

%% 3. 異なるサンプリング時間での離散化
T_list = [sampling_config.T_fast, sampling_config.T_default, sampling_config.T_slow];
T_names = {'高速', 'デフォルト', '低速'};
discrete_models = cell(length(T_list), 1);

for i = 1:length(T_list)
    T = T_list(i);
    fprintf('\n--- %sサンプリング (T = %g [s]) ---\n', T_names{i}, T);
    
    [Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T, sampling_config.method);
    
    % 離散化モデルの保存
    discrete_models{i} = struct();
    discrete_models{i}.Ad = Ad;
    discrete_models{i}.Bd = Bd;
    discrete_models{i}.Cd = Cd;
    discrete_models{i}.Dd = Dd;
    discrete_models{i}.T = T;
    discrete_models{i}.method = sampling_config.method;
end

%% 4. デフォルトサンプリング時間でのより詳細な解析
fprintf('\n=== デフォルトサンプリング時間での詳細解析 ===\n');
T_default = sampling_config.T_default;
[Ad_default, Bd_default, Cd_default, Dd_default] = discretize_linear_model(A, B, C, D, T_default);

%% 5. 異なる離散化手法の比較
fprintf('\n=== 異なる離散化手法の比較 ===\n');
methods = {'zoh', 'euler', 'tustin'};
method_names = {'ゼロ次ホールド', 'オイラー法', 'タスチン変換'};

discrete_methods = cell(length(methods), 1);

for i = 1:length(methods)
    fprintf('\n--- %s ---\n', method_names{i});
    [Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T_default, methods{i});
    
    discrete_methods{i} = struct();
    discrete_methods{i}.Ad = Ad;
    discrete_methods{i}.Bd = Bd;
    discrete_methods{i}.method = methods{i};
    discrete_methods{i}.eigenvalues = eig(Ad);
end

%% 6. 離散時間LQR制御器の設計
fprintf('\n=== 離散時間LQR制御器の設計 ===\n');

try
    % 重み行列（連続時間と同じ）
    Q = diag([...
        10*ones(1,3), ...    % ベース位置
        100*ones(1,3), ...   % ベース姿勢
        1*ones(1,6), ...     % 関節位置
        1*ones(1,3), ...     % ベース速度
        10*ones(1,3), ...    % ベース角速度
        0.1*ones(1,6)]);     % 関節速度
    
    R = 0.1 * eye(6);        % 制御入力の重み
    
    % 離散時間LQR
    [Kd, Sd, Pd] = dlqr(Ad_default, Bd_default, Q, R);
    
    fprintf('離散時間LQR制御器が設計されました\n');
    fprintf('制御ゲイン Kd のサイズ: %dx%d\n', size(Kd));
    
    % 閉ループ系の固有値
    Ad_cl = Ad_default - Bd_default * Kd;
    eigenvalues_cl_d = eig(Ad_cl);
    
    fprintf('離散時間閉ループ系の固有値（最初の10個）:\n');
    n_display = min(10, length(eigenvalues_cl_d));
    for i = 1:n_display
        magnitude = abs(eigenvalues_cl_d(i));
        if imag(eigenvalues_cl_d(i)) == 0
            fprintf('  λ%d = %7.4f (|λ| = %7.4f)\n', i, real(eigenvalues_cl_d(i)), magnitude);
        else
            fprintf('  λ%d = %7.4f + %7.4fi (|λ| = %7.4f)\n', i, real(eigenvalues_cl_d(i)), imag(eigenvalues_cl_d(i)), magnitude);
        end
    end
    
    % 安定性確認
    stable_d = all(abs(eigenvalues_cl_d) < 1);
    if stable_d
        fprintf('離散時間閉ループ系の安定性: 安定\n');
    else
        fprintf('離散時間閉ループ系の安定性: 不安定\n');
    end
    
catch ME
    warning('離散時間LQR制御器の設計でエラーが発生しました: %s', ME.message);
end

%% 7. 離散化モデルの保存
fprintf('\n=== 離散化モデルの保存 ===\n');

discrete_model_data = struct();
discrete_model_data.continuous = linear_model;
discrete_model_data.default = discrete_models{2};  % デフォルトサンプリング
discrete_model_data.sampling_config = sampling_config;
discrete_model_data.all_sampling_rates = discrete_models;
discrete_model_data.all_methods = discrete_methods;

if exist('Kd', 'var')
    discrete_model_data.lqr_controller = struct();
    discrete_model_data.lqr_controller.Kd = Kd;
    discrete_model_data.lqr_controller.Q = Q;
    discrete_model_data.lqr_controller.R = R;
end

save('discrete_model.mat', 'discrete_model_data');
fprintf('離散化モデルを discrete_model.mat に保存しました\n');

%% 8. ステップ応答の比較（連続時間 vs 離散時間）
fprintf('\n=== ステップ応答の比較 ===\n');

try
    % 時間軸
    t_final = 1.0;  % 1秒間
    
    % 連続時間システム
    sys_c = ss(A, B(:,1), C(1,:), D(1,1));  % 1入力1出力で簡単化
    t_c = 0:0.001:t_final;
    [y_c, t_c] = step(sys_c, t_c);
    
    % 離散時間システム
    sys_d = ss(Ad_default, Bd_default(:,1), Cd_default(1,:), Dd_default(1,1), T_default);
    t_d = 0:T_default:t_final;
    [y_d, t_d] = step(sys_d, t_d);
    
    fprintf('ステップ応答を計算しました\n');
    fprintf('  連続時間: %d点, 離散時間: %d点\n', length(y_c), length(y_d));
    
    % 簡単な比較
    if length(y_c) > 10 && length(y_d) > 5
        fprintf('  最終値比較: 連続 = %7.4f, 離散 = %7.4f\n', y_c(end), y_d(end));
    end
    
catch ME
    warning('ステップ応答の計算でエラーが発生しました: %s', ME.message);
end

%% 9. サンプリング時間変更の使用例
fprintf('\n=== サンプリング時間変更の使用例 ===\n');

% ユーザーが後で変更する場合の例
% 任意のサンプリング時間での離散化
T_custom = 0.003;  % 20ms (50Hz)

fprintf('カスタムサンプリング時間: T = %g [s]\n', T_custom);
[Ad_custom, Bd_custom, ~, ~] = discretize_linear_model(A, B, C, D, T_custom);
fprintf('カスタム離散化完了\n');

fprintf('\n=== 離散化テスト完了 ===\n');
fprintf('結果は discrete_model.mat に保存されています\n');
fprintf('\nサンプリング時間を変更したい場合は、以下のようにしてください:\n');
fprintf('  T_new = 0.02;  %% 新しいサンプリング時間\n');
fprintf('  [Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T_new);\n');