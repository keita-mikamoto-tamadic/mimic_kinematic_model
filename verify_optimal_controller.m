function verify_optimal_controller()
% 最適LQR制御器の検証
% 離散化システムに対する安定性と性能を確認

fprintf('=== 最適LQR制御器の検証 ===\n\n');

% 最適制御器の読み込み
if exist('optimal_lqr_controller.mat', 'file')
    load('optimal_lqr_controller.mat');
    K_opt = optimal_controller.K;
    Q_config = optimal_controller.Q_config;
    stability_margin = optimal_controller.stability_margin;
    T = optimal_controller.T;
    
    fprintf('最適制御器を読み込みました\n');
    fprintf('サンプリング時間: T = %g [s]\n', T);
    fprintf('安定性マージン: %7.4f\n', stability_margin);
else
    error('optimal_lqr_controller.mat が見つかりません');
end

% 離散化モデルの読み込み
if exist('discrete_model.mat', 'file')
    load('discrete_model.mat');
    Ad = discrete_model_data.default.Ad;
    Bd = discrete_model_data.default.Bd;
    Cd = discrete_model_data.default.Cd;
    Dd = discrete_model_data.default.Dd;
else
    error('discrete_model.mat が見つかりません');
end

%% 1. 閉ループ系の安定性確認
Ad_cl = Ad - Bd * K_opt;
eigenvalues_cl = eig(Ad_cl);

fprintf('\n=== 閉ループ系の特性 ===\n');
fprintf('閉ループ固有値の数: %d\n', length(eigenvalues_cl));

% 全固有値の確認
max_abs_eig = max(abs(eigenvalues_cl));
stable = all(abs(eigenvalues_cl) < 1);

fprintf('最大固有値絶対値: %7.4f\n', max_abs_eig);
if stable
    fprintf('安定性: 安定\n');
else
    fprintf('安定性: 不安定\n');
end
fprintf('安定性マージン: %7.4f\n', 1 - max_abs_eig);

% 不安定極の数
unstable_poles = sum(abs(eigenvalues_cl) >= 1);
if unstable_poles > 0
    fprintf('警告: %d個の不安定極があります\n', unstable_poles);
else
    fprintf('全ての極が安定です\n');
end

%% 2. 制御ゲインの特性
fprintf('\n=== 制御ゲインの特性 ===\n');
fprintf('制御ゲイン K のサイズ: %dx%d\n', size(K_opt));
fprintf('制御ゲインのノルム: %7.4f\n', norm(K_opt));
fprintf('最大ゲイン: %7.4f\n', max(K_opt(:)));
fprintf('最小ゲイン: %7.4f\n', min(K_opt(:)));

% 各関節に対するゲイン
fprintf('\n制御ゲインの詳細:\n');
joint_names = {'右股', '右膝', '右輪', '左股', '左膝', '左輪'};
for i = 1:6
    fprintf('  %s関節: ', joint_names{i});
    for j = 1:4:24  % 主要な状態変数のみ表示
        if j+3 <= 24
            fprintf('[%6.2f]', K_opt(i,j));
        end
    end
    fprintf('\n');
end

%% 3. ステップ応答の計算
fprintf('\n=== ステップ応答特性 ===\n');

try
    % 閉ループ系のシステム
    sys_cl = ss(Ad_cl, Bd, Cd, Dd, T);
    
    % 初期条件応答（小さな初期偏差からの復帰）
    x0 = zeros(24, 1);
    x0(5) = deg2rad(1);  % ベースY軸周りに1度の初期偏差
    
    % 時間軸
    t_sim = 0:T:2;  % 2秒間のシミュレーション
    
    % 初期条件応答
    [y_initial, t_initial] = initial(sys_cl, x0, t_sim);
    
    % 安定時間の推定（2%整定時間）
    settling_threshold = 0.02 * abs(x0(5));  % 初期値の2%
    
    if size(y_initial, 1) > 10
        % ベース姿勢角度（Y軸周り）の応答を見る
        attitude_response = y_initial(:, 5);  % ベース姿勢角
        
        % 整定時間の計算
        settled_indices = find(abs(attitude_response) < settling_threshold);
        if ~isempty(settled_indices)
            settling_time = t_initial(settled_indices(1));
            fprintf('2%%整定時間: %7.4f [s]\n', settling_time);
        else
            fprintf('2秒以内に整定していません\n');
        end
        
        % 最大オーバーシュート
        max_response = max(abs(attitude_response));
        overshoot = (max_response - abs(x0(5))) / abs(x0(5)) * 100;
        fprintf('最大オーバーシュート: %7.2f%%\n', overshoot);
        
        % 最終値
        final_value = attitude_response(end);
        fprintf('最終値: %7.4f [rad] (%7.2f [deg])\n', final_value, rad2deg(final_value));
        
    end
    
catch ME
    fprintf('ステップ応答の計算でエラー: %s\n', ME.message);
end

%% 4. 制御入力の大きさ確認
fprintf('\n=== 制御入力の特性 ===\n');

% 初期偏差に対する制御入力
u_initial = -K_opt * x0;

fprintf('初期偏差1度に対する制御入力:\n');
for i = 1:6
    fprintf('  %s関節: %7.4f [Nm]\n', joint_names{i}, u_initial(i));
end

max_control_effort = max(abs(u_initial));
fprintf('最大制御入力: %7.4f [Nm]\n', max_control_effort);

%% 5. 従来設定との比較
fprintf('\n=== 従来設定との比較 ===\n');

% 基本的なLQR設定
Q_basic = eye(24);
R_basic = eye(6);

try
    [K_basic, ~, ~] = dlqr(Ad, Bd, Q_basic, R_basic);
    Ad_cl_basic = Ad - Bd * K_basic;
    eig_basic = eig(Ad_cl_basic);
    stable_basic = all(abs(eig_basic) < 1);
    
    fprintf('基本設定:\n');
    if stable_basic
        fprintf('  安定性: 安定\n');
    else
        fprintf('  安定性: 不安定\n');
    end
    fprintf('  最大固有値: %7.4f\n', max(abs(eig_basic)));
    fprintf('  制御ゲインノルム: %7.4f\n', norm(K_basic));
    
    fprintf('最適設定:\n');
    if stable
        fprintf('  安定性: 安定\n');
    else
        fprintf('  安定性: 不安定\n');
    end
    fprintf('  最大固有値: %7.4f\n', max_abs_eig);
    fprintf('  制御ゲインノルム: %7.4f\n', norm(K_opt));
    
    improvement = stable && ~stable_basic;
    if improvement
        fprintf('改善: 成功\n');
    else
        fprintf('改善: 変化なし\n');
    end
    
catch ME
    fprintf('比較計算でエラー: %s\n', ME.message);
end

%% 6. 結論
fprintf('\n=== 検証結果の結論 ===\n');

if stable
    fprintf('✓ 最適LQR制御器により離散化システムが安定化されました\n');
    fprintf('✓ サンプリング時間 T = %g [s] で制御可能です\n', T);
    fprintf('✓ 安定性マージン %7.4f を確保しています\n', stability_margin);
    
    if max_control_effort < 100  % 100Nmを実用的な上限とする
        fprintf('✓ 制御入力は実用的な範囲内です\n');
    else
        fprintf('注意: 制御入力が大きいです（%7.1f Nm）\n', max_control_effort);
    end
    
else
    fprintf('✗ 制御器設計に問題があります\n');
end

fprintf('\n制御器の実装準備が完了しました\n');

end