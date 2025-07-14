function tune_lqr_weights()
% LQR重み行列の調整による安定化
% 離散化後の不安定システムに対してLQR制御器を設計

% 離散化モデルの読み込み
if exist('discrete_model.mat', 'file')
    load('discrete_model.mat');
    Ad = discrete_model_data.default.Ad;
    Bd = discrete_model_data.default.Bd;
    Cd = discrete_model_data.default.Cd;
    T = discrete_model_data.default.T;
    fprintf('離散化モデルを読み込みました (T = %g [s])\n', T);
else
    error('discrete_model.mat が見つかりません。先に run_discretization_test.m を実行してください');
end

n = size(Ad, 1);  % 状態数
m = size(Bd, 2);  % 入力数

fprintf('\n=== LQR重み行列の調整による安定化 ===\n');
fprintf('状態数: %d, 入力数: %d\n', n, m);

% 離散化システムの安定性確認
eigenvalues_open = eig(Ad);
unstable_poles = sum(abs(eigenvalues_open) > 1);
fprintf('開ループ系の不安定極数: %d\n', unstable_poles);

%% 1. 基本的な重み設定
fprintf('\n--- ケース1: 基本的な重み設定 ---\n');
Q1 = eye(n);          % 全状態に等しい重み
R1 = eye(m);          % 全入力に等しい重み

try
    [K1, S1, P1] = dlqr(Ad, Bd, Q1, R1);
    Ad_cl1 = Ad - Bd * K1;
    eigenvalues_cl1 = eig(Ad_cl1);
    stable1 = all(abs(eigenvalues_cl1) < 1);
    
    fprintf('制御ゲインのノルム: %7.4f\n', norm(K1));
    if stable1
        fprintf('閉ループ安定性: 安定\n');
    else
        fprintf('閉ループ安定性: 不安定\n');
    end
    fprintf('最大極の絶対値: %7.4f\n', max(abs(eigenvalues_cl1)));
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    stable1 = false;
end

%% 2. 姿勢に重点を置いた重み設定
fprintf('\n--- ケース2: 姿勢重視の重み設定 ---\n');
Q2 = diag([...
    1*ones(1,3), ...     % ベース位置（低重み）
    1000*ones(1,3), ...  % ベース姿勢（超高重み）
    1*ones(1,6), ...     % 関節位置（低重み）
    1*ones(1,3), ...     % ベース速度（低重み）
    100*ones(1,3), ...   % ベース角速度（高重み）
    1*ones(1,6)]);       % 関節速度（低重み）

R2 = 0.01 * eye(m);     % 制御入力の重み（小さく）

try
    [K2, S2, P2] = dlqr(Ad, Bd, Q2, R2);
    Ad_cl2 = Ad - Bd * K2;
    eigenvalues_cl2 = eig(Ad_cl2);
    stable2 = all(abs(eigenvalues_cl2) < 1);
    
    fprintf('制御ゲインのノルム: %7.4f\n', norm(K2));
    if stable2
        fprintf('閉ループ安定性: 安定\n');
    else
        fprintf('閉ループ安定性: 不安定\n');
    end
    fprintf('最大極の絶対値: %7.4f\n', max(abs(eigenvalues_cl2)));
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    stable2 = false;
end

%% 3. 非常に大きな状態重みと小さな入力重み
fprintf('\n--- ケース3: 超高状態重み、超低入力重み ---\n');
Q3 = diag([...
    100*ones(1,3), ...    % ベース位置
    10000*ones(1,3), ...  % ベース姿勢（超々高重み）
    10*ones(1,6), ...     % 関節位置
    10*ones(1,3), ...     % ベース速度
    1000*ones(1,3), ...   % ベース角速度（超高重み）
    1*ones(1,6)]);        % 関節速度

R3 = 0.001 * eye(m);      % 制御入力の重み（超小さく）

try
    [K3, S3, P3] = dlqr(Ad, Bd, Q3, R3);
    Ad_cl3 = Ad - Bd * K3;
    eigenvalues_cl3 = eig(Ad_cl3);
    stable3 = all(abs(eigenvalues_cl3) < 1);
    
    fprintf('制御ゲインのノルム: %7.4f\n', norm(K3));
    if stable3
        fprintf('閉ループ安定性: 安定\n');
    else
        fprintf('閉ループ安定性: 不安定\n');
    end
    fprintf('最大極の絶対値: %7.4f\n', max(abs(eigenvalues_cl3)));
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    stable3 = false;
end

%% 4. 反復的な重み調整
fprintf('\n--- ケース4: 反復的重み調整 ---\n');

% 重み調整のパラメータ
q_base_pos = [1, 10, 100, 1000];
q_base_att = [100, 1000, 10000, 100000];
r_input = [1, 0.1, 0.01, 0.001];

best_stable = false;
best_margin = 0;
best_K = [];
best_config = [];

fprintf('重み調整の探索中...\n');
total_combinations = length(q_base_pos) * length(q_base_att) * length(r_input);
count = 0;

for i = 1:length(q_base_pos)
    for j = 1:length(q_base_att)
        for k = 1:length(r_input)
            count = count + 1;
            
            Q_test = diag([...
                q_base_pos(i)*ones(1,3), ...  % ベース位置
                q_base_att(j)*ones(1,3), ...  % ベース姿勢
                1*ones(1,6), ...              % 関節位置
                1*ones(1,3), ...              % ベース速度
                10*ones(1,3), ...             % ベース角速度
                0.1*ones(1,6)]);              % 関節速度
            
            R_test = r_input(k) * eye(m);
            
            try
                [K_test, ~, ~] = dlqr(Ad, Bd, Q_test, R_test);
                Ad_cl_test = Ad - Bd * K_test;
                eigenvalues_test = eig(Ad_cl_test);
                
                max_abs_eig = max(abs(eigenvalues_test));
                stable_test = all(abs(eigenvalues_test) < 1);
                
                if stable_test
                    stability_margin = 1 - max_abs_eig;
                    if stability_margin > best_margin
                        best_stable = true;
                        best_margin = stability_margin;
                        best_K = K_test;
                        best_config = [q_base_pos(i), q_base_att(j), r_input(k)];
                        
                        fprintf('  改善: q_pos=%g, q_att=%g, r=%g → margin=%7.4f\n', ...
                            q_base_pos(i), q_base_att(j), r_input(k), stability_margin);
                    end
                end
                
            catch
                % LQR設計に失敗した場合はスキップ
            end
            
            if mod(count, 16) == 0
                fprintf('  進捗: %d/%d\n', count, total_combinations);
            end
        end
    end
end

%% 結果のまとめ
fprintf('\n=== 重み調整結果のまとめ ===\n');

results = [stable1, stable2, stable3, best_stable];
case_names = {'基本設定', '姿勢重視', '超高重み', '最適調整'};

for i = 1:length(results)
    if results(i)
        fprintf('%-10s: 安定\n', case_names{i});
    else
        fprintf('%-10s: 不安定\n', case_names{i});
    end
end

if best_stable
    fprintf('\n=== 最適重み設定 ===\n');
    fprintf('ベース位置重み: %g\n', best_config(1));
    fprintf('ベース姿勢重み: %g\n', best_config(2));
    fprintf('入力重み: %g\n', best_config(3));
    fprintf('安定性マージン: %7.4f\n', best_margin);
    fprintf('制御ゲインのノルム: %7.4f\n', norm(best_K));
    
    % 最適制御器の保存
    optimal_controller = struct();
    optimal_controller.K = best_K;
    optimal_controller.Q_config = best_config;
    optimal_controller.stability_margin = best_margin;
    optimal_controller.T = T;
    
    save('optimal_lqr_controller.mat', 'optimal_controller');
    fprintf('\n最適制御器を optimal_lqr_controller.mat に保存しました\n');
    
else
    fprintf('\n警告: 安定な制御器が見つかりませんでした\n');
    fprintf('推奨事項:\n');
    fprintf('1. より細かい重み調整\n');
    fprintf('2. より小さなサンプリング時間\n');
    fprintf('3. 平衡点の見直し\n');
end

end