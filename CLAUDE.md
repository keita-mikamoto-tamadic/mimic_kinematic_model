# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

このプロジェクトは脚車輪型倒立振子ロボットの動力学シミュレーションと制御システム設計のためのMATLAB実装です。シンボリック計算を使用せず、数値計算のみで効率的に運動方程式を導出し、線形化・離散化・最適制御器設計まで一貫して実行できます。

## 主要な実行コマンド

### 基本テスト実行
```matlab
% 運動学テスト
run_kinematics_test

% 動力学テスト（非制約）
run_dynamics_test

% 制約条件付き動力学テスト
run_constrained_dynamics_test
```

### 線形化・制御器設計
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

### パラメータ変更
```matlab
% パラメータ定義ファイルを編集
edit param_definition.m

% モデル再構築
model_definition_numeric
```

## システム構成

- **ベース本体**: 倒立振子として動作する胴体部（浮遊ベース、6DOF）
- **脚部**: 左右2本の脚（各脚2関節：股関節・膝関節）
- **車輪**: 各脚先端に取り付けられた駆動輪
- **合計6自由度**: 股関節×2 + 膝関節×2 + 車輪×2

## コードアーキテクチャ

### 1. 単位ベクトル法による運動方程式導出
本実装の核心は単位ベクトル法を使用した効率的な運動方程式の導出です：

- `forward_dynamics.m`: 各自由度に単位加速度を与えて質量行列を構築
- `inverse_dynamics_unit.m`: 微分運動学による速度・加速度の伝播計算
- `compute_differential_kinematics.m`: 関節軸の単位ベクトルによる運動伝播

### 2. 制約条件付き動力学
床接触制約を考慮した現実的な動力学計算：

- `compute_constraints.m`: 床接触制約条件（両車輪が床面に接触）
- `forward_dynamics_constrained.m`: ラグランジュ未定乗数法による制約付き順動力学
- `inverse_dynamics_constrained.m`: 制約付き逆動力学

### 3. 線形化・制御設計のフロー
```
平衡点計算 → 線形化 → 離散化 → LQR重み調整 → 制御器検証
```

- `compute_equilibrium_point.m`: 現実的な直立姿勢の平衡点計算
- `compute_linearization.m`: Taylor 1次展開による線形化
- `discretize_linear_model.m`: ゼロ次ホールド（ZOH）による離散化
- `tune_lqr_weights.m`: 反復的な重み調整による安定化

### 4. パラメータ管理
- `param_definition.m`: 物理パラメータの一元管理
  - ホイール半径: 38.975mm (実機仕様)
  - 質量・慣性パラメータ
  - リンク位置・重心位置
- `model_definition_numeric.m`: 統合モデル構造体作成

## 重要な技術的特徴

### ラグランジュ未定乗数法の実装
制約付き運動方程式：
```
[M(q)  -J^T(q)] [q̈]   [-C(q,q̇)q̇ - G(q) + τ]
[J(q)    0    ] [λ] = [-J̇(q,q̇)q̇           ]
```
- λ: ラグランジュ乗数（制約力の大きさ）
- J^T(q)λ: 一般化座標系での制約力
- 床接触の場合、λは床反力に対応

### LQR制御器設計
MATLABの`dlqr`関数を使用して離散時間リカッチ方程式を解く：
- 複数の重み設定で反復的に最適化
- 安定性マージンによる制御器選択
- 推奨サンプリング時間: T = 0.01s (100Hz)

## 注意事項

### MATLAB構文の制約
- ternary operator (`?:`) は使用不可、if-else文を使用
- 数値計算のみでシンボリック計算は避ける
- パス設定: `addpath('./function_utill')` が必要

### 実行順序の重要性
1. 基本テスト → 制約条件付きテスト → 線形化 → 離散化 → LQR調整 → 検証
2. パラメータ変更時は必ず `model_definition_numeric` を実行
3. LQR重み調整は安定性に決定的影響を与えるため必須

### 物理的整合性
- 床接触制約は必須（浮遊状態では物理的に無意味）
- ホイール半径は実機仕様（38.975mm）に設定済み
- 制約力の物理的意味を理解して実装されている

## 生成されるファイル
- `linear_model.mat`: 線形化モデル
- `discrete_model.mat`: 離散化モデル
- `optimal_lqr_controller.mat`: 最適制御器

これらのファイルは各段階の実行結果を保存し、次の段階で使用されます。