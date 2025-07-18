# 脚車輪型倒立振子ロボット動力学モデル

本プロジェクトは、脚車輪型倒立振子ロボットの動力学シミュレーションと制御システム設計のためのMATLAB実装です。シンボリック計算を使用せず、数値計算のみで効率的に運動方程式を導出し、線形化・離散化・最適制御器設計まで一貫して実行できます。

## 目次

1. [プロジェクト概要](#プロジェクト概要)
2. [システム構成](#システム構成)
3. [実行手順](#実行手順)
4. [ファイル構成](#ファイル構成)
5. [膝関節ミミック制約](#膝関節ミミック制約)
6. [理論的背景](#理論的背景)
7. [実装詳細](#実装詳細)
8. [制御器設計](#制御器設計)
9. [参考文献](#参考文献)

## プロジェクト概要

### ロボット構成
- **ベース本体**: 倒立振子として動作する胴体部
- **脚部**: 左右2本の脚（各脚2関節：股関節・膝関節）
- **車輪**: 各脚先端に取り付けられた駆動輪
- **実効自由度**: 股関節×2 + 車輪×2 = 4自由度（膝関節は股関節に従属）

### 主要機能
- 膝関節ミミック制約による自動膝トルク生成
- 床接触制約を考慮した現実的な動力学計算
- 数値ベースの高速運動方程式導出
- 平衡点周りの線形化
- 離散化とLQR最適制御器設計
- 安定性解析と制御性能評価

### 新機能：膝関節ミミック制約
- **制約条件**: `theta2 = 2*theta1`（膝関節角度は股関節角度の2倍）
- **膝トルク自動生成**: `tau_knee = 0.000718*theta1² + (-0.006939)*theta1 + 0.990022`
- **仮想仕事の原理**: 膝トルクが股関節の等価トルクに自動反映
- **制御自由度削減**: 12DOF → 10DOF（実効4DOF制御）

## システム構成

```
ベース本体 (浮遊ベース、6DOF)
├── 右脚
│   ├── 右股関節 (制御対象)
│   ├── 右膝関節 (追従動作: theta2 = 2*theta1)
│   └── 右車輪 (制御対象)
└── 左脚
    ├── 左股関節 (制御対象)
    ├── 左膝関節 (追従動作: theta2 = 2*theta1)
    └── 左車輪 (制御対象)
```

## 実行手順

### 【推奨】統合実行
```matlab
% 制御システム全体の統合実行（推奨）
run_system
```

**統合実行フロー（run_system.m）**：
1. **[1/3]** 削減モデルテスト実行
2. **[2/5]** 制御設計パラメータ設定（T, Q, R）
3. **[3/5]** 削減モデル制御器設計
4. **[4/5]** 100Hz制御サンプリング用離散化
5. **[5/5]** フィードバックゲイン表示と安定性確認

### 個別実行（詳細確認・デバッグ用）
```matlab
% 削減モデルテスト（膝トルク統合の確認）
run_reduced_model_test

% 削減モデル制御器設計
run_reduced_control_test

% 制御実装情報表示
display_control_implementation
```

### 従来システム（参考用）
```matlab
% 基本動力学テスト
run_dynamics_test

% 制約条件付き動力学テスト
run_constrained_dynamics_test

% 線形化実行
run_linearization_test

% 離散化実行
run_discretization_test

% LQR重み調整
tune_lqr_weights

% 制御器検証
verify_optimal_controller
```

### パラメータ変更方法

#### 制御パラメータ（推奨方法）
`run_system.m`の行16-26で直接設定変更：
```matlab
% サンプリング周期設定
T_sampling = 0.01;  % 100Hz制御サンプリング [s]

% LQR重み行列設定（20次元状態変数用）
Q_weights = [100, 100, 100, 50, 50, 50, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
%           [ベース位置×3, ベース姿勢×3, 関節位置×4, ベース速度×3, ベース角速度×3, 関節速度×4]

% 制御入力重み行列（4次元制御入力用）
R_weights = [1, 1, 1, 1];  % [右股関節, 右車輪, 左股関節, 左車輪]
```

#### 物理パラメータ変更時
```matlab
% パラメータ定義ファイルを編集
edit param_definition.m

% モデル再構築
model_definition_numeric

% システム再実行
run_system
```

## ファイル構成

### 基本設定ファイル
```
param_definition.m              # 物理パラメータ定義
model_definition_numeric.m      # 統合モデル構造体作成
model_definition_reduced.m      # 削減モデル構造体作成（膝制約対応）
```

### 運動学・動力学計算
```
compute_kinematics.m           # 前向き運動学
compute_kinematics_reduced.m   # 削減モデル運動学
compute_velocity.m             # 速度伝播計算
compute_velocity_reduced.m     # 削減モデル速度計算
inverse_dynamics.m             # 逆動力学（非制約）
forward_dynamics.m             # 順動力学（非制約）
```

### 制約条件付き動力学
```
compute_constraints.m          # 床接触制約条件
compute_joint_constraints.m    # 膝関節制約条件 (theta2 = 2*theta1)
compute_torque_mapping.m       # 膝トルク2次関数統合
forward_dynamics_constrained.m # 制約付き順動力学
inverse_dynamics_constrained.m # 制約付き逆動力学
```

### 削減モデル動力学
```
forward_dynamics_reduced.m           # 削減モデル順動力学
forward_dynamics_constrained_reduced.m # 削減モデル制約付き順動力学
inverse_dynamics_reduced.m           # 削減モデル逆動力学
inverse_dynamics_constrained_reduced.m # 削減モデル制約付き逆動力学
extract_reduced_states.m            # 状態変数削減
```

### 線形化・制御設計
```
compute_equilibrium_point.m    # 平衡点計算
compute_linearization.m        # 線形化（Taylor 1次展開）
discretize_linear_model.m      # 離散化（ZOH）
tune_lqr_weights.m            # LQR重み調整
verify_optimal_controller.m   # 制御器検証
```

### 統合実行システム
```
run_system.m                  # 統合実行ファイル（推奨）
run_reduced_model_test.m      # 削減モデルテスト
run_reduced_control_test.m    # 削減モデル制御器設計
display_control_implementation.m # 制御実装情報表示
discretize_linear_model.m     # 離散化関数（統合実行で使用）
```

### 従来システム（参考用）
```
run_dynamics_test.m           # 基本動力学テスト
run_constrained_dynamics_test.m # 制約条件付き動力学テスト
run_linearization_test.m      # 線形化テスト
run_discretization_test.m     # 離散化テスト
tune_lqr_weights.m           # LQR重み調整
verify_optimal_controller.m   # 制御器検証
```

### その他のユーティリティ
```
display_control_performance.m # 制御性能表示
display_robot_graph.m        # ロボット図表示
display_system_summary.m     # システム概要表示
show_results.m               # 結果表示
visualize_robot.m            # ロボット可視化
```

## 膝関節ミミック制約

### 1. 制約条件

膝関節角度制約：
$$\theta_{2,\mathrm{R}} = 2\theta_{1,\mathrm{R}}, \qquad \theta_{2,\mathrm{L}} = 2\theta_{1,\mathrm{L}}$$

速度制約：
$$\dot{\theta}_{2,\mathrm{R}} = 2\dot{\theta}_{1,\mathrm{R}}, \qquad \dot{\theta}_{2,\mathrm{L}} = 2\dot{\theta}_{1,\mathrm{L}}$$

加速度制約：
$$\ddot{\theta}_{2,\mathrm{R}} = 2\ddot{\theta}_{1,\mathrm{R}}, \qquad \ddot{\theta}_{2,\mathrm{L}} = 2\ddot{\theta}_{1,\mathrm{L}}$$

### 2. 膝トルクの2次関数

膝トルク自動生成：
$$\tau_{\mathrm{knee}} = 0.000718\,\theta_1^2 - 0.006939\,\theta_1 + 0.990022$$

### 3. 仮想仕事の原理による統合

仮想変位：$\delta\theta_1, \,\delta\theta_2 = 2\delta\theta_1$

仮想仕事：
$$\begin{align}
\delta W &= \tau_1\delta\theta_1 + \tau_2\delta\theta_2 \\
&= \tau_1\delta\theta_1 + \tau_2(2\delta\theta_1) \\
&= (\tau_1 + 2\tau_2)\delta\theta_1
\end{align}$$

等価トルク：
$$\tau_{\mathrm{equiv}} = \tau_1 + 2\tau_2$$

### 4. 制御自由度の削減
- **元の系**: 12DOF → 6個の関節トルク入力
- **削減系**: 10DOF → 4個の制御入力（股関節×2、車輪×2）
- **膝トルク**: 股関節角度に基づき自動生成
- **100Hz制御**: T=0.01s サンプリングで離散時間最適制御

## 理論的背景

### 1. 運動方程式の導出

本実装では**単位ベクトル法**を使用して効率的に運動方程式を導出します：

$\boldsymbol{M}(\boldsymbol{q})\ddot{\boldsymbol{q}} + \boldsymbol{C}(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}} + \boldsymbol{G}(\boldsymbol{q}) = \boldsymbol{\tau} + \boldsymbol{J}^{\mathsf{T}} \boldsymbol{\lambda}$

ここで：
- $\boldsymbol{M}(\boldsymbol{q}) \in \mathbb{R}^{n \times n}$: 慣性行列
- $\boldsymbol{C}(\boldsymbol{q},\dot{\boldsymbol{q}}) \in \mathbb{R}^{n \times n}$: コリオリ・遠心力項
- $\boldsymbol{G}(\boldsymbol{q}) \in \mathbb{R}^{n}$: 重力項
- $\boldsymbol{\tau} \in \mathbb{R}^{n}$: 関節トルク
- $\boldsymbol{J}^{\mathsf{T}} \boldsymbol{\lambda} \in \mathbb{R}^{n}$: 制約力

### 2. 制約条件

**床接触制約（両車輪が床面に接触）**：
$$\boldsymbol{\Phi}_{\mathrm{ground}}(\boldsymbol{q}) = \begin{bmatrix} z_{\mathrm{wheel,R}} \\ z_{\mathrm{wheel,L}} \end{bmatrix} = \boldsymbol{0}$$

**膝関節制約**：
$$\boldsymbol{\Phi}_{\mathrm{joint}}(\boldsymbol{q}) = \begin{bmatrix} \theta_{2,\mathrm{R}} - 2\theta_{1,\mathrm{R}} \\ \theta_{2,\mathrm{L}} - 2\theta_{1,\mathrm{L}} \end{bmatrix} = \boldsymbol{0}$$

制約の時間微分：
$$\boldsymbol{J}(\boldsymbol{q})\dot{\boldsymbol{q}} = \boldsymbol{0} \quad \text{（速度制約）}$$
$$\dot{\boldsymbol{J}}\dot{\boldsymbol{q}} + \boldsymbol{J}\ddot{\boldsymbol{q}} = \boldsymbol{0} \quad \text{（加速度制約）}$$

### 3. 制約付き運動方程式とラグランジュ未定乗数法

#### 3.1 制約条件の統合

床接触制約と膝関節制約を統合：
$$\boldsymbol{\Phi}_{\mathrm{total}}(\boldsymbol{q}) = \begin{bmatrix} 
z_{\mathrm{wheel,R}} - r_{\mathrm{wheel}} \\ 
z_{\mathrm{wheel,L}} - r_{\mathrm{wheel}} \\
\theta_{2,\mathrm{R}} - 2\theta_{1,\mathrm{R}} \\
\theta_{2,\mathrm{L}} - 2\theta_{1,\mathrm{L}}
\end{bmatrix} = \boldsymbol{0}$$

#### 3.2 制約付き運動方程式

$$\begin{bmatrix} 
\boldsymbol{M}(\boldsymbol{q}) & -\boldsymbol{J}^{\mathsf{T}}(\boldsymbol{q}) \\ 
\boldsymbol{J}(\boldsymbol{q}) & \boldsymbol{0} 
\end{bmatrix} 
\begin{bmatrix} 
\ddot{\boldsymbol{q}} \\ 
\boldsymbol{\lambda} 
\end{bmatrix} = 
\begin{bmatrix} 
-\boldsymbol{C}(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}} - \boldsymbol{G}(\boldsymbol{q}) + \boldsymbol{\tau} \\ 
-\dot{\boldsymbol{J}}(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}} 
\end{bmatrix}$$

#### 3.3 ラグランジュ乗数の物理的意味

- $\lambda_1$: 右車輪の床反力 $[\mathrm{N}]$
- $\lambda_2$: 左車輪の床反力 $[\mathrm{N}]$
- $\lambda_3$: 右膝関節の制約力 $[\mathrm{N \cdot m}]$
- $\lambda_4$: 左膝関節の制約力 $[\mathrm{N \cdot m}]$

### 4. 削減モデルの構築

#### 4.1 状態変数の削減
```matlab
% 元の状態: [base(6), theta1_R, theta2_R, wheel_R, theta1_L, theta2_L, wheel_L]
% 削減状態: [base(6), theta1_R, wheel_R, theta1_L, wheel_L]
% 削減率: 12 → 10 (16.7%削減)
```

#### 4.2 制約行列による拡張
```matlab
% 削減状態から完全状態への変換
C_expand = [eye(6),    zeros(6,4);     % ベース部分
           zeros(1,6), 1, 0, 0, 0;     % theta1_R
           zeros(1,6), 2, 0, 0, 0;     % theta2_R = 2*theta1_R
           zeros(1,6), 0, 1, 0, 0;     % wheel_R
           zeros(1,6), 0, 0, 1, 0;     % theta1_L
           zeros(1,6), 0, 0, 2, 0;     % theta2_L = 2*theta1_L
           zeros(1,6), 0, 0, 0, 1];    % wheel_L
```

### 5. 線形化

平衡点 $(\boldsymbol{q}_0, \dot{\boldsymbol{q}}_0 = \boldsymbol{0})$ 周りでTaylor 1次展開：

$\delta\dot{\boldsymbol{x}} = \boldsymbol{A} \delta\boldsymbol{x} + \boldsymbol{B} \delta\boldsymbol{u}$

削減モデルでは：
- 状態変数: $\boldsymbol{x} \in \mathbb{R}^{20}$（削減モデルの位置・速度）
- 入力変数: $\boldsymbol{u} \in \mathbb{R}^{6} \rightarrow \mathbb{R}^{4}$（33.3%削減）
- システム行列: $\boldsymbol{A} \in \mathbb{R}^{20 \times 20}$
- 入力行列: $\boldsymbol{B} \in \mathbb{R}^{20 \times 4}$

### 6. 離散化

ゼロ次ホールド（ZOH）による厳密離散化：

$\boldsymbol{x}[k+1] = \boldsymbol{A}_{\mathrm{d}} \boldsymbol{x}[k] + \boldsymbol{B}_{\mathrm{d}} \boldsymbol{u}[k]$

推奨サンプリング時間：
- **$T = 0.01\,\mathrm{s}$ (100Hz)**: 安定・実用的（推奨）
- **$T = 0.005\,\mathrm{s}$ (200Hz)**: より高精度

### 7. LQR最適制御

削減モデルでのコスト関数：$J = \sum_{k=0}^{\infty} \left( \boldsymbol{x}[k]^{\mathsf{T}} \boldsymbol{Q} \boldsymbol{x}[k] + \boldsymbol{u}[k]^{\mathsf{T}} \boldsymbol{R} \boldsymbol{u}[k] \right)$

最適フィードバックゲイン：
- $\boldsymbol{u} = -\boldsymbol{K} \boldsymbol{x}$
- $\boldsymbol{K} = \left(\boldsymbol{R} + \boldsymbol{B}_{\mathrm{d}}^{\mathsf{T}} \boldsymbol{P} \boldsymbol{B}_{\mathrm{d}}\right)^{-1} \boldsymbol{B}_{\mathrm{d}}^{\mathsf{T}} \boldsymbol{P} \boldsymbol{A}_{\mathrm{d}}$

制御入力：$\boldsymbol{u} = \begin{bmatrix} \tau_{1,\mathrm{R}} \\ \tau_{\mathrm{wheel,R}} \\ \tau_{1,\mathrm{L}} \\ \tau_{\mathrm{wheel,L}} \end{bmatrix}$

## 実装詳細

### 1. 膝トルク統合の流れ

1. **制約条件の設定**（`compute_joint_constraints.m`）
   - `theta2 = 2*theta1`制約の実装
   - 速度・加速度制約の時間微分

2. **トルク統合**（`compute_torque_mapping.m`）
   - 膝トルクの2次関数計算
   - 仮想仕事の原理による等価トルク統合

3. **動力学計算**（`*_reduced.m`）
   - 削減モデルでの順動力学・逆動力学
   - 制約条件の数値的検証

### 2. 制約検証システム

```matlab
% 制約条件の満足度確認
constraint_error_R = abs(ddtheta2_R - 2*ddtheta1_R);
constraint_error_L = abs(ddtheta2_L - 2*ddtheta1_L);
tolerance = 1e-6;

if constraint_error_R > tolerance
    warning('右脚の加速度制約条件が満たされていません');
end
```

### 3. 削減モデルの利点

- **計算効率**: 制御入力が6→4に削減
- **物理的整合性**: 膝トルクが股関節角度に自動追従
- **制御安定性**: 制約条件により予測可能な動作
- **実装簡易性**: 制御器設計が4入力システムで完結

## 制御器設計

### 1. 平衡点の設定

削減モデルの平衡点：
```matlab
% 平衡点（直立姿勢）
q_eq = zeros(10, 1);  % [ベース位置(3), ベース姿勢(3), 関節角度(4)]
q_eq(3) = base_height;  % ベースの高さ
q_eq(7) = theta1_eq;    % 股関節角度
q_eq(8) = wheel_radius; % 右車輪地面接触
q_eq(9) = theta1_eq;    % 股関節角度
q_eq(10) = wheel_radius;% 左車輪地面接触
```

### 2. LQR重み調整

削減モデル用の重み設定：
```matlab
% 状態重み行列 Q（20×20）
Q = diag([
    1000*ones(1,3), ...    % ベース位置（高重み）
    10000*ones(1,3), ...   % ベース姿勢（超高重み）
    1*ones(1,4), ...       % 関節位置（低重み）
    1*ones(1,3), ...       % ベース速度（低重み）
    100*ones(1,3), ...     % ベース角速度（高重み）
    1*ones(1,4)]);         % 関節速度（低重み）

% 入力重み行列 R（4×4）
R = 0.001 * eye(4);        % 制御入力（超低重み）
```

### 3. 制御実装

#### 3.1 制御入力の計算
$\boldsymbol{u} = -\boldsymbol{K} (\boldsymbol{x} - \boldsymbol{x}_{\mathrm{eq}}) + \boldsymbol{u}_{\mathrm{eq}}$

**詳細：**
- $\boldsymbol{x} \in \mathbb{R}^{20}$: 現在の状態変数
- $\boldsymbol{x}_{\mathrm{eq}} \in \mathbb{R}^{20}$: 平衡点状態変数
- $(\boldsymbol{x} - \boldsymbol{x}_{\mathrm{eq}}) \in \mathbb{R}^{20}$: 状態変数の偏差
- $\boldsymbol{K} \in \mathbb{R}^{4 \times 20}$: フィードバックゲイン行列
- $-\boldsymbol{K} (\boldsymbol{x} - \boldsymbol{x}_{\mathrm{eq}}) \in \mathbb{R}^{4}$: 偏差に基づくフィードバック制御量
- $\boldsymbol{u}_{\mathrm{eq}} \in \mathbb{R}^{4}$: 平衡点制御入力
- $\boldsymbol{u} \in \mathbb{R}^{4}$: 最終制御入力

#### 3.2 制御の物理的意味
1. **状態偏差の検出**: (x - x_eq) で目標状態からのずれを計算
2. **フィードバック制御**: -K で偏差に比例した制御量を生成
3. **平衡点補償**: u_eq で重力補償などの定常制御入力を加算

#### 3.3 実装例
```matlab
% 制御器初期化
load('reduced_lqr_controller.mat');
K = reduced_controller.K;          % 4×20ゲイン行列
x_eq = reduced_controller.x0;      % 平衡点状態
u_eq = reduced_controller.u0;      % 平衡点制御入力

% 制御ループ
while running
    x_current = get_robot_state();      % 現在状態取得 (20次元)
    u = -K * (x_current - x_eq) + u_eq; % 制御入力計算 (4次元)
    apply_control(u);                   % 制御入力適用
    pause(0.01);                        % 100Hz制御
end
```

#### 3.4 制御入力の構成
削減モデルの制御入力（4次元）：
$$\boldsymbol{u} = \begin{bmatrix} 
\tau_{1,\mathrm{R,equiv}} \\ 
\tau_{\mathrm{wheel,R}} \\ 
\tau_{1,\mathrm{L,equiv}} \\ 
\tau_{\mathrm{wheel,L}} 
\end{bmatrix}$$

ここで：
- $\tau_{1,\mathrm{R,equiv}}$: 右脚股関節等価トルク（膝トルク統合済み）$[\mathrm{N \cdot m}]$
- $\tau_{\mathrm{wheel,R}}$: 右脚車輪トルク $[\mathrm{N \cdot m}]$
- $\tau_{1,\mathrm{L,equiv}}$: 左脚股関節等価トルク（膝トルク統合済み）$[\mathrm{N \cdot m}]$
- $\tau_{\mathrm{wheel,L}}$: 左脚車輪トルク $[\mathrm{N \cdot m}]$

### 4. 制御性能（統合実行システム）

`run_system.m`による統合実行システムの性能指標：

#### 4.1 制御システム仕様
- **制御入力数**: $6 \rightarrow 4$（33.3%削減）
- **状態変数**: $\boldsymbol{x} \in \mathbb{R}^{20}$（削減モデル位置・速度）
- **フィードバックゲイン**: $\boldsymbol{K} \in \mathbb{R}^{4 \times 20}$
- **サンプリング**: $100\,\mathrm{Hz}$ ($T=0.01\,\mathrm{s}$) 離散時間制御

#### 4.2 安定性保証
- **極配置**: 20個の閉ループ極すべて単位円内 ($|\lambda_i| < 1, \, \forall i$)
- **自動検証**: 各極の絶対値と安定性判定を自動表示
- **膝トルク**: 股関節角度に基づく2次関数で自動生成

#### 4.3 実装効率
- **一括実行**: 5段階プロセスの自動化
- **パラメータ統合**: $T, \boldsymbol{Q}, \boldsymbol{R}$ の一元管理
- **計算効率**: 制御則計算の高速化

## 注意事項

1. **制約条件の収束**: 膝トルク統合により制約条件の収束が完全でない場合がある
2. **制約検証**: 警告メッセージは膝トルク統合の期待される動作
3. **パラメータ調整**: 膝トルクの2次関数係数は実機に合わせて調整可能
4. **MATLAB構文**: ternary operator (`?:`) は使用不可、if-else文を使用
5. **数値計算**: シンボリック計算を避け、数値計算で高速化
6. **制約処理**: 床接触制約と膝関節制約の両方が必須
7. **LQR調整**: 削減モデル用の重み行列調整が安定性に決定的影響
8. **サンプリング時間**: 高速サンプリングが離散化安定性に重要

## 参考文献

1. Robotics: Modelling, Planning and Control - Siciliano et al.
2. Modern Control Engineering - Ogata
3. Optimal Control Theory - Kirk
4. Robot Dynamics and Control - Spong & Vidyasagar
5. Constrained Dynamics and Control - Blajer

---

## ライセンス

このプロジェクトは教育・研究目的で作成されています。

## 更新履歴

- 2024-07: 初版作成
- 2024-07: 制約条件付き動力学実装
- 2024-07: 線形化・LQR制御器設計完成
- 2024-07: ホイール半径を実機仕様（38.975mm）に更新、ラグランジュ未定乗数法の詳細解説を追加
- 2024-07: 膝関節ミミック制約統合、膝トルク2次関数自動生成機能追加
- 2024-07: 統合実行ファイル`run_system.m`実装、100Hz制御サンプリング・LQR極安定性確認機能追加