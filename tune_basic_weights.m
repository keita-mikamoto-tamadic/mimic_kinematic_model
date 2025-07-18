function [Q_optimal, R_optimal, K_optimal, stability_info] = tune_basic_weights(Ad, Bd, model_info)
% 基本的な重み設定による LQR 制御器設計
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

fprintf('--- 基本的な重み設定による LQR 制御器設計 ---\n');

n = size(Ad, 1);  % 状態数 (20)
m = size(Bd, 2);  % 入力数 (4)

% 基本的な重み設定: 全状態・入力に等しい重み
Q_optimal = eye(n);
R_optimal = eye(m);

fprintf('重み設定:\n');
fprintf('  状態重み Q: 単位行列 (全状態に等重み)\n');
fprintf('  入力重み R: 単位行列 (全入力に等重み)\n');

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
    stability_info.method = 'basic';
    
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
    stability_info.method = 'basic';
    stability_info.error = ME.message;
end

fprintf('基本重み設定による制御器設計完了\n\n');

end