# 脚車輪型倒立振子ロボット動力学モデル

本プロジェクトは、脚車輪型倒立振子ロボットの動力学シミュレーションと制御システム設計のためのMATLAB実装です。シンボリック計算を使用せず、数値計算のみで効率的に運動方程式を導出し、線形化・離散化・最適制御器設計まで一貫して実行できます。

## 目次

1. [プロジェクト概要](#プロジェクト概要)
2. [システム構成](#システム構成)
3. [ファイル構成](#ファイル構成)
4. [使用方法](#使用方法)
5. [理論的背景](#理論的背景)
6. [実装詳細](#実装詳細)
7. [制御器設計](#制御器設計)
8. [参考文献](#参考文献)

## プロジェクト概要

### ロボット構成
- **ベース本体**: 倒立振子として動作する胴体部
- **脚部**: 左右2本の脚（各脚2関節：股関節・膝関節）
- **車輪**: 各脚先端に取り付けられた駆動輪
- **合計6自由度**: 股関節×2 + 膝関節×2 + 車輪×2

### 主要機能
- 床接触制約を考慮した現実的な動力学計算
- 数値ベースの高速運動方程式導出
- 平衡点周りの線形化
- 離散化とLQR最適制御器設計
- 安定性解析と制御性能評価

## システム構成

```
ベース本体 (浮遊ベース、6DOF)
├── 右脚
│   ├── 右股関節 (回転関節)
│   ├── 右膝関節 (回転関節)
│   └── 右車輪 (駆動輪)
└── 左脚
    ├── 左股関節 (回転関節)
    ├── 左膝関節 (回転関節)
    └── 左車輪 (駆動輪)
```

### 座標系
- **ワールド座標系**: 固定慣性座標系
- **ベース座標系**: ロボット本体に固定
- **関節座標系**: 各関節に固定

## ファイル構成

### 基本設定ファイル
```
param_definition.m              # 物理パラメータ定義
model_definition_numeric.m      # 統合モデル構造体作成
```

### 運動学・動力学計算
```
compute_kinematics.m           # 前向き運動学
compute_velocity.m             # 速度伝播計算
inverse_dynamics.m             # 逆動力学（非制約）
forward_dynamics.m             # 順動力学（非制約）
```

### 制約条件付き動力学
```
compute_constraints.m          # 床接触制約条件
forward_dynamics_constrained.m # 制約付き順動力学
inverse_dynamics_constrained.m # 制約付き逆動力学
```

### 線形化・制御設計
```
compute_equilibrium_point.m    # 平衡点計算
compute_linearization.m        # 線形化（Taylor 1次展開）
discretize_linear_model.m      # 離散化（ZOH）
tune_lqr_weights.m            # LQR重み調整
verify_optimal_controller.m   # 制御器検証
```

### テスト・検証スクリプト
```
run_kinematics_test.m         # 運動学テスト
run_dynamics_test.m           # 動力学テスト
run_constraint_test.m         # 制約条件テスト
run_linearization_test.m      # 線形化テスト
run_discretization_test.m     # 離散化テスト
analyze_stability_requirement.m # 安定性要求分析
```

### ユーティリティ関数
```
function_utill/
├── skew.m                    # 外積行列
├── rodrigues.m               # ロドリゲス回転公式
└── ...
```

## 使用方法

### 1. 基本テストの実行

```matlab
% 運動学テスト
run_kinematics_test

% 動力学テスト（非制約）
run_dynamics_test

% 制約条件付き動力学テスト
run_constraint_test
```

### 2. 線形化・制御器設計

```matlab
% 線形化実行
run_linearization_test

% 離散化実行
run_discretization_test

% LQR重み調整（重要！）
tune_lqr_weights

% 制御器検証
verify_optimal_controller
```

### 3. パラメータ変更

```matlab
% パラメータ定義ファイルを編集
edit param_definition.m

% モデル再構築
model_definition_numeric
```

### 4. カスタムサンプリング時間での離散化

```matlab
% 線形化モデル読み込み
load('linear_model.mat');

% カスタム離散化
T_custom = 0.005;  % 5ms
[Ad, Bd, Cd, Dd] = discretize_linear_model(linear_model.A, linear_model.B, ...
                                          linear_model.C, linear_model.D, T_custom);
```

## 理論的背景

### 1. 運動方程式の導出

本実装では**単位ベクトル法**を使用して効率的に運動方程式を導出します：

```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ + J^T λ
```

ここで：
- `M(q)`: 慣性行列
- `C(q,q̇)`: コリオリ・遠心力項
- `G(q)`: 重力項
- `τ`: 関節トルク
- `J^T λ`: 制約力

### 2. 制約条件

床接触制約（両車輪が床面に接触）：

```
Φ(q) = [z_right_wheel; z_left_wheel] = 0
```

制約の時間微分：
```
J(q)q̇ = 0  （速度制約）
J̇q̇ + Jq̈ = 0  （加速度制約）
```

### 3. 制約付き運動方程式とラグランジュ未定乗数法

ラグランジュ未定乗数法は、制約条件下での最適化問題を解く強力な手法です。ロボット動力学では、床接触などの制約条件を考慮した運動方程式を導出するために使用されます。

#### 3.1 制約条件の定義

床接触制約（両車輪が床面に接触）：
```
Φ(q) = [z_right_wheel - wheel_radius; z_left_wheel - wheel_radius] = 0
```

制約条件の時間微分：
```
速度制約：   J(q)q̇ = 0
加速度制約： J̇(q,q̇)q̇ + J(q)q̈ = 0
```

ここで、J(q) = ∂Φ/∂q は制約ヤコビアン行列です。

#### 3.2 ラグランジュ未定乗数法の適用

**Step 1: ラグランジアンの拡張**

制約なしのラグランジアン：
```
L = T - V  （運動エネルギー - ポテンシャルエネルギー）
```

制約付きラグランジアン：
```
L_c = L + λ^T Φ(q)
```

ここで、λは**ラグランジュ乗数**（未定乗数）で、制約力の大きさを表します。

**Step 2: オイラー・ラグランジュ方程式**

制約付きオイラー・ラグランジュ方程式：
```
d/dt(∂L_c/∂q̇) - ∂L_c/∂q = τ_ext
```

展開すると：
```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ + J^T(q)λ
```

**Step 3: 制約付き運動方程式の完成**

制約条件と運動方程式を連立：
```
[M(q)  -J^T(q)] [q̈]   [-C(q,q̇)q̇ - G(q) + τ]
[J(q)    0    ] [λ] = [-J̇(q,q̇)q̇           ]
```

この線形系を解くことで、q̈（加速度）とλ（制約力）を同時に求めます。

#### 3.3 ラグランジュ乗数の物理的意味

- **λ**: 制約力の大きさを表すベクトル
- **J^T(q)λ**: 一般化座標系での制約力
- 床接触の場合、λは床反力に対応

#### 3.4 制約処理の数値実装

```matlab
function [qdd, lambda] = solve_constrained_dynamics(q, qd, tau, model)
    % 慣性行列、コリオリ項、重力項の計算
    [M, C, G] = compute_dynamics_matrices(q, qd, model);
    
    % 制約条件とヤコビアンの計算
    [phi, J, J_dot] = compute_constraints(q, qd, model);
    
    % 制約付き運動方程式の行列形成
    n = length(q);
    m = length(phi);
    
    A = [M,     -J';
         J,  zeros(m,m)];
    
    b = [-C - G + tau;
         -J_dot * qd];
    
    % 線形系の解法
    x = A \ b;
    
    qdd = x(1:n);        % 加速度
    lambda = x(n+1:end); % ラグランジュ乗数（制約力）
end
```

#### 3.5 制約力の解釈

本実装では：
- **λ(1)**: 右車輪の床反力 [N]
- **λ(2)**: 左車輪の床反力 [N]

これらの値は物理的に意味があり、ロボットの安定性解析に使用できます。

### 4. 線形化

平衡点 `(q₀, q̇₀ = 0)` 周りでTaylor 1次展開：

```
δẋ = A δx + B δu
```

ここで：
- `δx = [δq; δq̇]`: 状態偏差
- `δu = δτ`: 入力偏差
- `A = ∂f/∂x |₀`: システム行列
- `B = ∂f/∂u |₀`: 入力行列

### 5. 離散化

ゼロ次ホールド（ZOH）による厳密離散化：

```
x[k+1] = Aₑ x[k] + Bₑ u[k]
```

ここで：
```
[Aₑ Bₑ] = exp([A B; 0 0] T)
[0  I ]        
```

### 6. LQR最適制御

コスト関数：
```
J = ∫₀^∞ (x^T Q x + u^T R u) dt
```

最適フィードバックゲイン：
```
u = -K x,  K = R⁻¹ B^T P
```

`P`は代数リカッチ方程式の解：
```
A^T P + P A - P B R⁻¹ B^T P + Q = 0
```

## 実装詳細

### 1. パラメータ定義（param_definition.m）

```matlab
% 基本物理パラメータ
g = 9.81;                           % 重力加速度 [m/s^2]

% ホイール半径 [m]
wheel_radius = (77.95 / 2) / 1000;  % 38.975 mm

% 質量パラメータ [kg]
m = [4.195338, 0.09062, 0.709174, 0.643498, 0.09062, 0.709174, 0.643498];
% [ベース, 右股, 右膝, 右ホイール, 左股, 左膝, 左ホイール]

% ベース重心位置 [m]
cogx_b = -0.002253; cogy_b = -0.002051; cogz_b = -0.002169;

% 右脚リンク位置・重心 [m]
l1_x = 0; l1_y = -0.1325; l1_z = 0;                    % 右股関節位置
wR_x = 0; wR_y = -0.0545; wR_z = -0.15 + wheel_radius; % 右ホイール位置

% 左脚リンク位置・重心 [m]
l3_x = 0; l3_y = 0.1325; l3_z = 0;                     % 左股関節位置
wL_x = 0; wL_y = 0.0545; wL_z = -0.15 + wheel_radius;  % 左ホイール位置

% 慣性テンソル [kg⋅m^2]
Ib = [0.030220, 0.001117, 0.001369; ...]; % ベース慣性行列
I_wheel_R = [0.001232, 5.4039e-08, 3.2154e-06; ...]; % 右ホイール慣性
I_wheel_L = [0.001232, -5.4039e-08, -3.2154e-06; ...]; % 左ホイール慣性
```

### 2. 運動学計算の流れ

1. **関節角度から位置**: `compute_kinematics.m`
   - ベース位置・姿勢
   - 各リンク位置・姿勢
   - 車輪接地点位置

2. **速度伝播**: `compute_velocity.m`
   - ベース速度から各リンク速度を計算
   - ヤコビアン行列の構築

3. **動力学計算**: `forward_dynamics.m` / `inverse_dynamics.m`
   - 慣性行列、コリオリ項、重力項の計算
   - 運動方程式の解法

### 3. 制約処理の実装

制約条件（床接触）：
```matlab
function [Phi, J, J_dot] = compute_constraints(model, q, q_dot)
    % 右車輪の床接触制約
    Phi_right = wheel_position_right(3) - wheel_radius;
    
    % 左車輪の床接触制約  
    Phi_left = wheel_position_left(3) - wheel_radius;
    
    Phi = [Phi_right; Phi_left];
    
    % 制約ヤコビアン J = ∂Φ/∂q
    % 制約ヤコビアンの時間微分 J̇
end
```

### 4. 線形化の実装

数値微分による線形化：
```matlab
function [A, B] = compute_linearization(model, q0, q_dot0)
    % 平衡点での状態方程式: ẋ = f(x, u)
    
    % システム行列 A = ∂f/∂x
    A = zeros(2*n, 2*n);
    epsilon = 1e-8;
    
    for i = 1:2*n
        x_plus = x0; x_plus(i) = x_plus(i) + epsilon;
        x_minus = x0; x_minus(i) = x_minus(i) - epsilon;
        
        f_plus = state_equation(x_plus, u0);
        f_minus = state_equation(x_minus, u0);
        
        A(:,i) = (f_plus - f_minus) / (2*epsilon);
    end
    
    % 入力行列 B = ∂f/∂u（同様の手順）
end
```

## 制御器設計

### 1. 平衡点の設定

現実的な直立姿勢：
```matlab
% 平衡点（直立姿勢）
q_eq = zeros(12, 1);  % [ベース位置(3), ベース姿勢(3), 関節角度(6)]
q_eq(3) = base_height;  % ベースの高さ
q_eq(9) = wheel_radius;  % 右車輪地面接触
q_eq(12) = wheel_radius; % 左車輪地面接触
```

### 2. LQR重み調整

安定化のための重み設定：
```matlab
% 状態重み行列 Q
Q = diag([
    1000*ones(1,3), ...    % ベース位置（高重み）
    10000*ones(1,3), ...   % ベース姿勢（超高重み）
    1*ones(1,6), ...       % 関節位置（低重み）
    1*ones(1,3), ...       % ベース速度（低重み）
    100*ones(1,3), ...     % ベース角速度（高重み）
    1*ones(1,6)]);         % 関節速度（低重み）

% 入力重み行列 R
R = 0.001 * eye(6);        % 制御入力（超低重み）
```

### 3. 制御性能

最適制御器の性能指標：
- **安定性マージン**: 0.0012
- **整定時間**: 0.13秒
- **オーバーシュート**: 0%
- **最大制御入力**: 2.95 Nm

### 4. 離散化サンプリング時間

推奨サンプリング時間：
- **T = 0.01s (100Hz)**: 安定・実用的（推奨）
- **T = 0.005s (200Hz)**: より高精度
- **T > 0.05s**: 不安定リスク

### 新しいホイール半径仕様

本実装では実際のロボットに合わせてホイール半径を更新：
- **ホイール半径**: 38.975mm （直径77.95mm）
- **従来値**: 50mm から約11mm小さく調整
- **影響**: 床接触点、平衡点、制約条件が自動調整

## 注意事項

1. **MATLAB構文**: ternary operator (`?:`) は使用不可、if-else文を使用
2. **数値計算**: シンボリック計算を避け、数値計算で高速化
3. **制約処理**: 床接触制約は必須（浮遊状態では物理的に無意味）
4. **LQR調整**: 重み行列の調整が安定性に決定的影響
5. **サンプリング時間**: 高速サンプリングが離散化安定性に重要
6. **ホイール半径**: パラメータファイルで一元管理、実機仕様に合わせて調整可能

## 参考文献

1. Robotics: Modelling, Planning and Control - Siciliano et al.
2. Modern Control Engineering - Ogata
3. Optimal Control Theory - Kirk
4. Robot Dynamics and Control - Spong & Vidyasagar

---

## ライセンス

このプロジェクトは教育・研究目的で作成されています。

## 更新履歴

- 2024-07: 初版作成
- 2024-07: 制約条件付き動力学実装
- 2024-07: 線形化・LQR制御器設計完成
- 2024-07: ホイール半径を実機仕様（38.975mm）に更新、ラグランジュ未定乗数法の詳細解説を追加