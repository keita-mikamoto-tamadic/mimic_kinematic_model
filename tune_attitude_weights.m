function [Q_optimal, R_optimal, K_optimal, stability_info] = tune_attitude_weights(Ad, Bd, model_info)
% 姿勢重視の重み設定による LQR 制御器設計
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

fprintf('--- 姿勢重視の重み設定による LQR 制御器設計 ---\n');

n = size(Ad, 1);  % 状態数 (20)
m = size(Bd, 2);  % 入力数 (4)

% 姿勢重視の重み設定
% 削減モデルの状態変数構成 (20次元):
% [ベース位置(3), ベース姿勢(3), 関節位置(4), ベース速度(3), ベース角速度(3), 関節速度(4)]
Q_weights = [...
    10*ones(1,3), ...      % ベース位置 (低重み)
    1000*ones(1,3), ...    % ベース姿勢 (超高重み) - 姿勢安定化重視
    1*ones(1,4), ...       % 関節位置 (低重み)
    1*ones(1,3), ...       % ベース速度 (低重み)
    100*ones(1,3), ...     % ベース角速度 (高重み) - 姿勢安定化重視
    1*ones(1,4)];          % 関節速度 (低重み)

Q_optimal = diag(Q_weights);

% 制御入力重み (小さく設定して制御性能を向上)
R_optimal = 0.01 * eye(m);

fprintf('重み設定:\n');
fprintf('  ベース位置重み: %.0f\n', Q_weights(1));
fprintf('  ベース姿勢重み: %.0f (高重み)\n', Q_weights(4));
fprintf('  関節位置重み: %.0f\n', Q_weights(7));
fprintf('  ベース速度重み: %.0f\n', Q_weights(11));
fprintf('  ベース角速度重み: %.0f (高重み)\n', Q_weights(14));
fprintf('  関節速度重み: %.0f\n', Q_weights(17));
fprintf('  制御入力重み: %.3f (低重み)\n', R_optimal(1,1));

try
    % LQR制御器設計
    [K_optimal, S, P] = dlqr(Ad, Bd, Q_optimal, R_optimal);
    
    % 閉ループ系の計算
    Ad_cl = Ad - Bd * K_optimal;
    eigenvalues_cl = eig(Ad_cl);
    
    % 安定性解析
    max_eigenvalue = max(abs(eigenvalues_cl));
    is_stable = all(abs(eigenvalues_cl) < 1);
    stability_margin = 1 - max_eigenvalue;
    
    % 制御性能指標
    control_norm = norm(K_optimal);
    
    % 安定性情報の構造化
    stability_info = struct();
    stability_info.is_stable = is_stable;
    stability_info.max_eigenvalue = max_eigenvalue;
    stability_info.stability_margin = stability_margin;
    stability_info.control_norm = control_norm;
    stability_info.eigenvalues = eigenvalues_cl;
    stability_info.method = 'attitude';
    stability_info.Q_weights = Q_weights;
    stability_info.R_weight = R_optimal(1,1);
    
    % 結果表示
    fprintf('制御器設計結果:\n');
    fprintf('  制御ゲインのノルム: %7.4f\n', control_norm);
    fprintf('  最大極の絶対値: %7.4f\n', max_eigenvalue);
    
    if is_stable
        fprintf('  閉ループ安定性: 安定\n');
        fprintf('  安定性マージン: %7.4f\n', stability_margin);
    else
        fprintf('  閉ループ安定性: 不安定\n');
        fprintf('  不安定極数: %d\n', sum(abs(eigenvalues_cl) >= 1));
    end
    
catch ME
    fprintf('エラー: %s\n', ME.message);
    
    % エラー時のデフォルト値
    K_optimal = zeros(m, n);
    stability_info = struct();
    stability_info.is_stable = false;
    stability_info.max_eigenvalue = inf;
    stability_info.stability_margin = -inf;
    stability_info.control_norm = 0;
    stability_info.eigenvalues = [];
    stability_info.method = 'attitude';
    stability_info.error = ME.message;
end

fprintf('姿勢重視重み設定による制御器設計完了\n\n');

end