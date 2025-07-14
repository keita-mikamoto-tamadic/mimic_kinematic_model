clear; clc;

% パスの設定
addpath('./function_utill');

% モデル定義
model = model_definition_numeric();

% 現実的な初期姿勢の設定（両輪が床面に接触）
q = zeros(12,1);
q(1:3) = [0; 0; 0.3];                   % ベース位置（床面から30cm上）
q(4:6) = deg2rad([0; 0; 0]);            % ベース姿勢（水平）
q(7) = deg2rad(30);                     % 右脚股関節
q(8) = deg2rad(-60);                    % 右脚膝関節
q(10) = deg2rad(30);                    % 左脚股関節
q(11) = deg2rad(-60);                   % 左脚膝関節

% 一般化速度の設定
dq = zeros(12,1);

% 運動学計算と3D可視化
kin = compute_kinematics(q, model);
visualize_robot(kin, model);

%% 1. 重力補償トルクの計算
[tau_gravity, f_ground_static] = compute_gravity_compensation(q, model);

%% 2. 重力補償トルクを与えた場合の動力学（理想的には静的平衡）
fprintf('\n=== 重力補償トルクを与えた場合 ===\n');
[ddq_gravity, lambda_gravity] = forward_dynamics_constrained(q, dq, tau_gravity, model);

fprintf('重力補償トルク適用時の加速度:\n');
fprintf('  ベース線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_gravity(1:3));
fprintf('  ベース角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_gravity(4:6));
fprintf('  右脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_gravity(7:9));
fprintf('  左脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_gravity(10:12));
fprintf('床反力 [N]: [%7.4f, %7.4f]\n', lambda_gravity);

%% 3. 重力補償トルクの逆トルクを与えた場合（自由落下に近い状態）
fprintf('\n=== 重力補償トルクの逆トルクを与えた場合 ===\n');
tau_anti_gravity = -tau_gravity;  % 逆トルク

[ddq_anti, lambda_anti] = forward_dynamics_constrained(q, dq, tau_anti_gravity, model);

fprintf('逆重力補償トルク適用時の加速度:\n');
fprintf('  ベース線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_anti(1:3));
fprintf('  ベース角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_anti(4:6));
fprintf('  右脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_anti(7:9));
fprintf('  左脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_anti(10:12));
fprintf('床反力 [N]: [%7.4f, %7.4f]\n', lambda_anti);

%% 4. 姿勢制御のシナリオ：前方に傾いた場合の復元トルク
fprintf('\n=== 姿勢制御シナリオ ===\n');

% 前方に5度傾いた姿勢
q_tilted = q;
q_tilted(5) = deg2rad(5);  % Y軸周りに5度回転（前方に傾く）

% 傾いた姿勢での重力補償トルク
[tau_tilted, f_tilted] = compute_gravity_compensation(q_tilted, model);

% 復元トルク：元の姿勢に戻すための追加トルク
tau_restore = tau_tilted - tau_gravity;

fprintf('前方5度傾斜時の追加復元トルク [Nm]:\n');
fprintf('  右脚: [%7.4f, %7.4f, %7.4f]\n', tau_restore(1:3));
fprintf('  左脚: [%7.4f, %7.4f, %7.4f]\n', tau_restore(4:6));

% 復元トルクを適用した場合の動力学
[ddq_restore, lambda_restore] = forward_dynamics_constrained(q_tilted, dq, tau_tilted, model);

fprintf('復元トルク適用時の加速度:\n');
fprintf('  ベース線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_restore(1:3));
fprintf('  ベース角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_restore(4:6));
fprintf('床反力 [N]: [%7.4f, %7.4f]\n', lambda_restore);

%% 5. 前進運動のシナリオ：前方加速のための関節トルク
fprintf('\n=== 前進運動シナリオ ===\n');

% 目標：前方に0.5m/s²で加速
ddq_forward = zeros(12,1);
ddq_forward(1) = 0.5;  % X方向加速

[tau_forward, f_forward] = inverse_dynamics_constrained(q, dq, ddq_forward, model);

fprintf('前方0.5m/s²加速のための関節トルク [Nm]:\n');
fprintf('  右脚: [%7.4f, %7.4f, %7.4f]\n', tau_forward(1:3));
fprintf('  左脚: [%7.4f, %7.4f, %7.4f]\n', tau_forward(4:6));
fprintf('必要な床反力 [N]: [%7.4f, %7.4f]\n', f_forward);

% 検証
[ddq_forward_verify, lambda_forward_verify] = forward_dynamics_constrained(q, dq, tau_forward, model);
fprintf('実際の前方加速度: %7.4f m/s² (目標: 0.5)\n', ddq_forward_verify(1));
fprintf('誤差: %7.4f m/s²\n', abs(ddq_forward_verify(1) - 0.5));