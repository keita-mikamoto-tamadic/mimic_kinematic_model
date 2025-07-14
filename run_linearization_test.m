clear; clc;

% パスの設定
addpath('./function_utill');

% モデル定義
model = model_definition_numeric();

%% 1. 平衡点の計算
fprintf('=== 脚車輪型倒立振子の線形化 ===\n\n');

% 平衡点の設定
config = struct();
config.base_height = 0.25;           % ベース高さ
config.base_orientation = [0; 0; 0]; % 直立姿勢
config.hip_angle = deg2rad(25);      % 股関節角度
config.knee_angle = deg2rad(-50);    % 膝関節角度
config.wheel_angle = 0;              % ホイール角度

[q0, dq0, tau0] = compute_equilibrium_point(model, config);

%% 2. 運動学可視化
fprintf('\n=== 平衡点での運動学可視化 ===\n');
kin = compute_kinematics(q0, model);
visualize_robot(kin, model);

%% 3. テイラー1次展開による線形化
fprintf('\n=== テイラー1次展開による線形化 ===\n');
[A, B, C, D] = compute_linearization(q0, dq0, tau0, model);

%% 4. 状態方程式の保存
fprintf('\n=== 状態方程式の保存 ===\n');
linear_model = struct();
linear_model.A = A;
linear_model.B = B;
linear_model.C = C;
linear_model.D = D;
linear_model.q0 = q0;
linear_model.dq0 = dq0;
linear_model.tau0 = tau0;
linear_model.model = model;

save('linear_model.mat', 'linear_model');
fprintf('線形化モデルを linear_model.mat に保存しました\n');

%% 5. 線形化モデルの検証
fprintf('\n=== 線形化モデルの検証 ===\n');

% 小さな摂動を与えて線形モデルと非線形モデルの応答を比較
delta_q = 0.01 * ones(12, 1);    % 位置の摂動
delta_dq = 0.01 * ones(12, 1);   % 速度の摂動
delta_tau = 0.1 * ones(6, 1);    % トルクの摂動

% 摂動した状態
q_pert = q0 + delta_q;
dq_pert = dq0 + delta_dq;
tau_pert = tau0 + delta_tau;

% 非線形モデルの応答
try
    [ddq_nonlinear, ~] = forward_dynamics_constrained(q_pert, dq_pert, tau_pert, model);
    dx_nonlinear = [dq_pert; ddq_nonlinear];
    
    % 線形モデルの応答
    delta_x = [delta_q; delta_dq];
    delta_u = delta_tau;
    dx_linear = A * delta_x + B * delta_u;
    
    % 比較
    error_norm = norm(dx_nonlinear - dx_linear);
    fprintf('線形化誤差のノルム: %e\n', error_norm);
    
    if error_norm < 1e-2
        fprintf('線形化の精度: 良好\n');
    elseif error_norm < 1e-1
        fprintf('線形化の精度: 普通\n');
    else
        fprintf('線形化の精度: 注意が必要\n');
    end
    
    % 詳細な比較
    fprintf('\n位置微分の比較:\n');
    fprintf('  非線形モデル: max = %7.4f, min = %7.4f\n', ...
        max(dx_nonlinear(1:12)), min(dx_nonlinear(1:12)));
    fprintf('  線形モデル:   max = %7.4f, min = %7.4f\n', ...
        max(dx_linear(1:12)), min(dx_linear(1:12)));
    
    fprintf('\n加速度の比較:\n');
    fprintf('  非線形モデル: max = %7.4f, min = %7.4f\n', ...
        max(dx_nonlinear(13:24)), min(dx_nonlinear(13:24)));
    fprintf('  線形モデル:   max = %7.4f, min = %7.4f\n', ...
        max(dx_linear(13:24)), min(dx_linear(13:24)));
    
catch ME
    warning('線形化モデルの検証でエラーが発生しました: %s', ME.message);
end

%% 6. 制御設計への応用例
fprintf('\n=== 制御設計への応用例 ===\n');

% LQR制御器の設計例
try
    % 重み行列の設定
    Q = diag([...
        10*ones(1,3), ...    % ベース位置
        100*ones(1,3), ...   % ベース姿勢（特に重要）
        1*ones(1,6), ...     % 関節位置
        1*ones(1,3), ...     % ベース速度
        10*ones(1,3), ...    % ベース角速度
        0.1*ones(1,6)]);     % 関節速度
    
    R = 0.1 * eye(6);        % 制御入力の重み
    
    % LQR制御器の設計
    [K, S, P] = lqr(A, B, Q, R);
    
    fprintf('LQR制御器が設計されました\n');
    fprintf('制御ゲイン K のサイズ: %dx%d\n', size(K));
    fprintf('制御ゲイン K の最大値: %7.4f\n', max(K(:)));
    fprintf('制御ゲイン K の最小値: %7.4f\n', min(K(:)));
    
    % 閉ループ系の固有値
    A_cl = A - B * K;
    eigenvalues_cl = eig(A_cl);
    fprintf('\n閉ループ系の固有値:\n');
    for i = 1:length(eigenvalues_cl)
        if imag(eigenvalues_cl(i)) == 0
            fprintf('  λ%d = %7.4f\n', i, real(eigenvalues_cl(i)));
        else
            fprintf('  λ%d = %7.4f + %7.4fi\n', i, real(eigenvalues_cl(i)), imag(eigenvalues_cl(i)));
        end
    end
    
    % 安定性の確認
    stable = all(real(eigenvalues_cl) < 0);
    if stable
        fprintf('閉ループ系の安定性: 安定\n');
    else
        fprintf('閉ループ系の安定性: 不安定\n');
    end
    
catch ME
    warning('LQR制御器の設計でエラーが発生しました: %s', ME.message);
end

fprintf('\n=== 線形化完了 ===\n');
fprintf('結果は linear_model.mat に保存されています\n');