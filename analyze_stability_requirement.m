function analyze_stability_requirement(linear_model_file)
% 離散化での安定性要求を分析
% 
% 入力:
%   linear_model_file: 線形化モデルファイル名 (デフォルト: 'linear_model.mat')

if nargin < 1
    linear_model_file = 'linear_model.mat';
end

% 線形化モデルの読み込み
if exist(linear_model_file, 'file')
    load(linear_model_file);
    A = linear_model.A;
    B = linear_model.B;
    C = linear_model.C;
    D = linear_model.D;
else
    error('線形化モデルファイルが見つかりません: %s', linear_model_file);
end

fprintf('=== 離散化安定性要求の分析 ===\n\n');

% 連続時間システムの固有値分析
eigenvalues = eig(A);
real_parts = real(eigenvalues);
imag_parts = imag(eigenvalues);

fprintf('=== 連続時間システムの固有値分析 ===\n');
unstable_poles = real_parts > 0;
n_unstable = sum(unstable_poles);

fprintf('不安定極の数: %d\n', n_unstable);

if n_unstable > 0
    max_unstable_pole = max(real_parts);
    fprintf('最大不安定極: λ_max = %7.4f\n', max_unstable_pole);
    
    % サンプリング時間の要求
    T_theoretical = 1 / max_unstable_pole;
    T_practical_1 = 0.1 / max_unstable_pole;  % 10%の余裕
    T_practical_2 = 0.01 / max_unstable_pole; % 1%の余裕
    
    fprintf('\n=== サンプリング時間の要求 ===\n');
    fprintf('理論的限界: T < %7.5f [s] (%7.1f Hz)\n', T_theoretical, 1/T_theoretical);
    fprintf('実用的要求1: T < %7.5f [s] (%7.1f Hz) [10%%余裕]\n', T_practical_1, 1/T_practical_1);
    fprintf('実用的要求2: T < %7.5f [s] (%7.1f Hz) [1%%余裕]\n', T_practical_2, 1/T_practical_2);
    
    % 現在のサンプリング時間での離散化極
    T_test = [0.01, 0.005, 0.003, 0.001, 0.0005];
    
    fprintf('\n=== 各サンプリング時間での最大不安定極 ===\n');
    for i = 1:length(T_test)
        T = T_test(i);
        discrete_pole = exp(max_unstable_pole * T);
        stable = discrete_pole < 1;
        
        if stable
            fprintf('T = %7.4f [s]: 離散化極 = %7.4f (安定)\n', T, discrete_pole);
        else
            fprintf('T = %7.4f [s]: 離散化極 = %7.4f (不安定)\n', T, discrete_pole);
        end
    end
    
    % 安定化に必要な最小サンプリング周波数
    T_required = log(0.99) / max_unstable_pole;  % 離散化極 < 0.99
    if T_required > 0
        fprintf('\n安定化に必要なサンプリング時間: T < %7.5f [s] (%7.1f Hz)\n', ...
            abs(T_required), 1/abs(T_required));
    end
end

%% 制御可能性の確認
fprintf('\n=== 制御可能性の確認 ===\n');
try
    Co = ctrb(A, B);
    rank_Co = rank(Co);
    n_states = size(A, 1);
    
    fprintf('可制御性行列のランク: %d / %d\n', rank_Co, n_states);
    
    if rank_Co == n_states
        fprintf('システムは完全可制御です\n');
    else
        fprintf('システムは部分可制御です（問題あり）\n');
        
        % 可制御でない極を特定
        [Ac, Bc, Cc, T, k] = ctrbf(A, B, C);
        controllable_states = k(1);
        uncontrollable_states = n_states - controllable_states;
        
        fprintf('可制御状態数: %d\n', controllable_states);
        fprintf('不可制御状態数: %d\n', uncontrollable_states);
        
        if uncontrollable_states > 0
            % 不可制御部分の固有値
            Auc = Ac(controllable_states+1:end, controllable_states+1:end);
            uncontrollable_poles = eig(Auc);
            unstable_uncontrollable = sum(real(uncontrollable_poles) > 0);
            
            fprintf('不可制御かつ不安定な極の数: %d\n', unstable_uncontrollable);
            
            if unstable_uncontrollable > 0
                fprintf('警告: 不可制御な不安定極があります！制御不可能です\n');
            end
        end
    end
catch ME
    fprintf('可制御性の確認でエラー: %s\n', ME.message);
end

%% 推奨解決策
fprintf('\n=== 推奨解決策 ===\n');

if n_unstable > 0
    fprintf('1. 非常に高速なサンプリング (T < %g [s]) を使用\n', T_practical_2);
    fprintf('2. 予備安定化制御を実装\n');
    fprintf('3. 平衡点周りでの線形近似を改善\n');
    fprintf('4. より安定な平衡点を探索\n');
    
    % 実用的なサンプリング時間の提案
    T_suggested = min(T_practical_2, 0.001);
    fprintf('\n推奨サンプリング時間: T = %g [s] (%g Hz)\n', T_suggested, 1/T_suggested);
    
    % テスト用離散化
    fprintf('\n=== 推奨サンプリング時間でのテスト ===\n');
    [Ad, Bd, Cd, Dd] = discretize_linear_model(A, B, C, D, T_suggested);
    
    discrete_eigenvalues = eig(Ad);
    unstable_discrete = sum(abs(discrete_eigenvalues) > 1);
    
    fprintf('推奨T=%gでの不安定離散極数: %d\n', T_suggested, unstable_discrete);
    
    if unstable_discrete == 0
        fprintf('成功: 全ての離散極が安定です\n');
    else
        fprintf('注意: まだ%d個の不安定極があります\n', unstable_discrete);
    end
end

end