function [Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T, method)
% 連続時間状態方程式の離散化
% dx/dt = Ax + Bu, y = Cx + Du を離散化して
% x[k+1] = Ad*x[k] + Bd*u[k], y[k] = Cd*x[k] + Dd*u[k] を求める
%
% 入力:
%   A, B, C, D : 連続時間システム行列
%   T          : サンプリング時間 [s]
%   method     : 離散化手法 ('zoh', 'euler', 'tustin')
%
% 出力:
%   Ad, Bd, Cd, Dd : 離散化システム行列

if nargin < 6
    method = 'zoh';  % デフォルトはゼロ次ホールド
end

fprintf('=== 状態方程式の離散化 ===\n');
fprintf('サンプリング時間: T = %g [s]\n', T);
fprintf('離散化手法: %s\n', method);

n = size(A, 1);  % 状態変数の数
m = size(B, 2);  % 入力変数の数

switch lower(method)
    case 'zoh'
        % ゼロ次ホールド（Zero-Order Hold）による離散化
        % 最も一般的で正確な手法
        fprintf('ゼロ次ホールド法による離散化...\n');
        
        % 行列指数関数を用いた厳密解
        % [Ad Bd] = exp([A B; 0 0]*T)
        %  [0  I ]
        
        % 拡張行列の構築
        F = [A, B; zeros(m, n), zeros(m, m)];
        
        % 行列指数関数の計算
        expF = expm(F * T);
        
        % 離散化行列の抽出
        Ad = expF(1:n, 1:n);
        Bd = expF(1:n, n+1:n+m);
        
    case 'euler'
        % オイラー法（前進差分）
        % 簡単だが精度は低い
        fprintf('オイラー法による離散化...\n');
        
        Ad = eye(n) + A * T;
        Bd = B * T;
        
    case 'tustin'
        % タスチン変換（双一次変換）
        % s = 2/T * (z-1)/(z+1) による変換
        fprintf('タスチン変換による離散化...\n');
        
        I = eye(n);
        Ad = (I + A*T/2) / (I - A*T/2);
        Bd = (I - A*T/2) \ (B*T);
        
    otherwise
        error('未対応の離散化手法です: %s', method);
end

% 出力行列は通常変化しない
Cd = C;
Dd = D;

% 離散化結果の表示
fprintf('離散化完了\n');
fprintf('  連続時間システム: %dx%d (状態), %dx%d (入力)\n', size(A), size(B));
fprintf('  離散時間システム: %dx%d (状態), %dx%d (入力)\n', size(Ad), size(Bd));

% 安定性の確認
eigenvalues_c = eig(A);
eigenvalues_d = eig(Ad);

fprintf('\n=== 安定性の比較 ===\n');

% 連続時間システムの安定性
stable_c = all(real(eigenvalues_c) < 0);
if stable_c
    fprintf('連続時間システム: 安定\n');
else
    fprintf('連続時間システム: 不安定\n');
end

% 離散時間システムの安定性
stable_d = all(abs(eigenvalues_d) < 1);
if stable_d
    fprintf('離散時間システム: 安定\n');
else
    fprintf('離散時間システム: 不安定\n');
end

% 危険な固有値の警告
unstable_poles_c = sum(real(eigenvalues_c) > 0);
unstable_poles_d = sum(abs(eigenvalues_d) > 1);

if unstable_poles_c > 0
    fprintf('警告: 連続時間システムに%d個の不安定極があります\n', unstable_poles_c);
end

if unstable_poles_d > 0
    fprintf('警告: 離散時間システムに%d個の不安定極があります\n', unstable_poles_d);
end

% サンプリング時間の妥当性チェック
max_eigenvalue = max(real(eigenvalues_c));
if max_eigenvalue > 0
    suggested_T = 0.1 / max_eigenvalue;  % 経験的な目安
    if T > suggested_T
        fprintf('注意: サンプリング時間が大きすぎる可能性があります\n');
        fprintf('      推奨サンプリング時間: T < %g [s]\n', suggested_T);
    end
end

% 固有値の詳細表示（最初の10個まで）
fprintf('\n主要な固有値の比較:\n');
n_display = min(10, length(eigenvalues_c));

for i = 1:n_display
    if imag(eigenvalues_c(i)) == 0
        fprintf('  連続: λ%d = %7.4f', i, real(eigenvalues_c(i)));
    else
        fprintf('  連続: λ%d = %7.4f + %7.4fi', i, real(eigenvalues_c(i)), imag(eigenvalues_c(i)));
    end
    
    if imag(eigenvalues_d(i)) == 0
        fprintf('  → 離散: λ%d = %7.4f\n', i, real(eigenvalues_d(i)));
    else
        fprintf('  → 離散: λ%d = %7.4f + %7.4fi\n', i, real(eigenvalues_d(i)), imag(eigenvalues_d(i)));
    end
end

if length(eigenvalues_c) > n_display
    fprintf('  ... (残り%d個の固有値は省略)\n', length(eigenvalues_c) - n_display);
end

end