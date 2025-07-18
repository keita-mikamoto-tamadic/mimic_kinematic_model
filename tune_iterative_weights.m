function [Q_optimal, R_optimal, K_optimal, stability_info] = tune_iterative_weights(Ad, Bd, model_info)
% 反復的重み調整による最適 LQR 制御器設計
% 
% 入力:
%   Ad: 離散化システム行列 (20x20)
%   Bd: 離散化入力行列 (20x4)
%   model_info: モデル情報構造体
%
% 出力:
%   Q_optimal: 最適状態重み行列 (20x20)
%   R_optimal: 最適入力重み行列 (4x4)
%   K_optimal: 最適制御ゲイン行列 (4x20)
%   stability_info: 安定性情報構造体

fprintf('--- 反復的重み調整による最適 LQR 制御器設計 ---\n');

n = size(Ad, 1);  % 状態数 (20)
m = size(Bd, 2);  % 入力数 (4)

% 重み調整のパラメータ範囲
q_base_pos = [1, 10, 100, 1000];                  % ベース位置重み
q_base_att = [100, 1000, 10000, 100000];          % ベース姿勢重み
q_joint_pos = [0.1, 1, 10, 100];                  % 関節位置重み
q_base_vel = [0.1, 1, 10, 100];                   % ベース速度重み
q_base_omega = [1, 10, 100, 1000];                % ベース角速度重み
q_joint_vel = [0.01, 0.1, 1, 10];                 % 関節速度重み
r_input = [0.001, 0.01, 0.1, 1];                  % 制御入力重み

% 最適化変数の初期化
best_stable = false;
best_margin = -inf;
best_K = [];
best_Q = [];
best_R = [];
best_config = [];

fprintf('重み調整パラメータ:\n');
fprintf('  ベース位置重み候補: %s\n', mat2str(q_base_pos));
fprintf('  ベース姿勢重み候補: %s\n', mat2str(q_base_att));
fprintf('  関節位置重み候補: %s\n', mat2str(q_joint_pos));
fprintf('  ベース速度重み候補: %s\n', mat2str(q_base_vel));
fprintf('  ベース角速度重み候補: %s\n', mat2str(q_base_omega));
fprintf('  関節速度重み候補: %s\n', mat2str(q_joint_vel));
fprintf('  制御入力重み候補: %s\n', mat2str(r_input));

% 全組み合わせ数の計算
total_combinations = length(q_base_pos) * length(q_base_att) * length(q_joint_pos) * ...
                    length(q_base_vel) * length(q_base_omega) * length(q_joint_vel) * ...
                    length(r_input);

fprintf('探索する組み合わせ数: %d\n', total_combinations);
fprintf('重み調整の探索を開始します...\n');

count = 0;
stable_count = 0;

% 反復的重み調整
for i1 = 1:length(q_base_pos)
    for i2 = 1:length(q_base_att)
        for i3 = 1:length(q_joint_pos)
            for i4 = 1:length(q_base_vel)
                for i5 = 1:length(q_base_omega)
                    for i6 = 1:length(q_joint_vel)
                        for i7 = 1:length(r_input)
                            count = count + 1;
                            
                            % 重み行列の構築
                            Q_test = diag([...
                                q_base_pos(i1)*ones(1,3), ...    % ベース位置
                                q_base_att(i2)*ones(1,3), ...    % ベース姿勢
                                q_joint_pos(i3)*ones(1,4), ...  % 関節位置
                                q_base_vel(i4)*ones(1,3), ...   % ベース速度
                                q_base_omega(i5)*ones(1,3), ... % ベース角速度
                                q_joint_vel(i6)*ones(1,4)]);    % 関節速度
                            
                            R_test = r_input(i7) * eye(m);
                            
                            try
                                % LQR制御器設計
                                [K_test, ~, ~] = dlqr(Ad, Bd, Q_test, R_test);
                                
                                % 閉ループ系の計算
                                Ad_cl_test = Ad - Bd * K_test;
                                eigenvalues_test = eig(Ad_cl_test);
                                
                                % 安定性解析
                                max_abs_eig = max(abs(eigenvalues_test));
                                stable_test = all(abs(eigenvalues_test) < 1);
                                
                                if stable_test
                                    stable_count = stable_count + 1;
                                    stability_margin = 1 - max_abs_eig;
                                    control_norm = norm(K_test);
                                    
                                    % 制御入力制限チェック
                                    max_control_gain = max(max(abs(K_test)));
                                    control_feasible = max_control_gain < 10000;  % 実用的な制御入力制限
                                    
                                    % 総合評価指標 (安定性マージン - 制御入力ペナルティ)
                                    if control_feasible
                                        performance_index = stability_margin - 0.0001 * control_norm;
                                    else
                                        performance_index = stability_margin - 0.01 * control_norm;
                                    end
                                    
                                    % 最適解の更新
                                    if performance_index > best_margin
                                        best_stable = true;
                                        best_margin = performance_index;
                                        best_K = K_test;
                                        best_Q = Q_test;
                                        best_R = R_test;
                                        best_config = [q_base_pos(i1), q_base_att(i2), q_joint_pos(i3), ...
                                                      q_base_vel(i4), q_base_omega(i5), q_joint_vel(i6), ...
                                                      r_input(i7)];
                                        
                                        fprintf('  改善: [%g,%g,%g,%g,%g,%g,%g] → margin=%7.4f, norm=%7.2f\n', ...
                                            q_base_pos(i1), q_base_att(i2), q_joint_pos(i3), ...
                                            q_base_vel(i4), q_base_omega(i5), q_joint_vel(i6), ...
                                            r_input(i7), stability_margin, control_norm);
                                    end
                                end
                                
                            catch
                                % LQR設計に失敗した場合はスキップ
                            end
                            
                            % 進捗表示
                            if mod(count, 256) == 0
                                fprintf('  進捗: %d/%d (%.1f%%), 安定解数: %d\n', ...
                                    count, total_combinations, 100*count/total_combinations, stable_count);
                            end
                        end
                    end
                end
            end
        end
    end
end

% 最適化結果の出力
fprintf('\n=== 反復的重み調整結果 ===\n');
fprintf('探索した組み合わせ数: %d\n', count);
fprintf('安定な制御器数: %d\n', stable_count);

if best_stable
    Q_optimal = best_Q;
    R_optimal = best_R;
    K_optimal = best_K;
    
    % 最終的な安定性解析
    Ad_cl = Ad - Bd * K_optimal;
    eigenvalues_cl = eig(Ad_cl);
    max_eigenvalue = max(abs(eigenvalues_cl));
    stability_margin = 1 - max_eigenvalue;
    control_norm = norm(K_optimal);
    
    % 安定性情報の構造化
    stability_info = struct();
    stability_info.is_stable = true;
    stability_info.max_eigenvalue = max_eigenvalue;
    stability_info.stability_margin = stability_margin;
    stability_info.control_norm = control_norm;
    stability_info.eigenvalues = eigenvalues_cl;
    stability_info.method = 'iterative';
    stability_info.best_config = best_config;
    stability_info.performance_index = best_margin;
    stability_info.total_combinations = total_combinations;
    stability_info.stable_count = stable_count;
    
    fprintf('\n最適重み設定:\n');
    fprintf('  ベース位置重み: %g\n', best_config(1));
    fprintf('  ベース姿勢重み: %g\n', best_config(2));
    fprintf('  関節位置重み: %g\n', best_config(3));
    fprintf('  ベース速度重み: %g\n', best_config(4));
    fprintf('  ベース角速度重み: %g\n', best_config(5));
    fprintf('  関節速度重み: %g\n', best_config(6));
    fprintf('  制御入力重み: %g\n', best_config(7));
    
    fprintf('\n制御器設計結果:\n');
    fprintf('  制御ゲインのノルム: %7.4f\n', control_norm);
    fprintf('  最大極の絶対値: %7.4f\n', max_eigenvalue);
    fprintf('  安定性マージン: %7.4f\n', stability_margin);
    fprintf('  性能指標: %7.4f\n', best_margin);
    
else
    % 最適解が見つからない場合のデフォルト値
    fprintf('\n警告: 安定な制御器が見つかりませんでした\n');
    
    Q_optimal = eye(n);
    R_optimal = eye(m);
    K_optimal = zeros(m, n);
    
    stability_info = struct();
    stability_info.is_stable = false;
    stability_info.max_eigenvalue = inf;
    stability_info.stability_margin = -inf;
    stability_info.control_norm = 0;
    stability_info.eigenvalues = [];
    stability_info.method = 'iterative';
    stability_info.total_combinations = total_combinations;
    stability_info.stable_count = stable_count;
    stability_info.warning = 'No stable controller found';
end

fprintf('反復的重み調整による制御器設計完了\n\n');

end