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

%% 拘束条件の確認
[phi, J_phi] = compute_constraints(q, dq, model);
fprintf('=== 拘束条件の確認 ===\n');
fprintf('拘束条件 phi = [%7.4f, %7.4f]\n', phi);
fprintf('拘束条件のヤコビアン J_phi サイズ: %dx%d\n', size(J_phi));

%% 拘束条件下での順動力学テスト
fprintf('\n=== 拘束条件下での順動力学テスト ===\n');

% ケース1: 関節トルクなし（静的平衡）
tau1 = zeros(6,1);
[ddq1, lambda1] = forward_dynamics_constrained(q, dq, tau1, model);

fprintf('ケース1: 関節トルクなし\n');
fprintf('  ベース加速度:\n');
fprintf('    線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq1(1:3));
fprintf('    角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq1(4:6));
fprintf('  関節加速度:\n');
fprintf('    右脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq1(7:9));
fprintf('    左脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq1(10:12));
fprintf('  床反力 [N]: [%7.4f, %7.4f]\n', lambda1);


%% 逆動力学テスト：目標加速度から関節トルクを計算
fprintf('\n=== 逆動力学テスト（床反力考慮） ===\n');

% 目標加速度：ベースを前方に0.1m/s²加速
ddq_target = zeros(12,1);
ddq_target(1) = 0.1;  % X方向に加速

[tau_req, f_ground] = inverse_dynamics_constrained(q, dq, ddq_target, model);

fprintf('目標加速度: ベースX方向 0.1 m/s²\n');
fprintf('  必要な関節トルク [Nm]:\n');
fprintf('    右脚: [%7.4f, %7.4f, %7.4f]\n', tau_req(1:3));
fprintf('    左脚: [%7.4f, %7.4f, %7.4f]\n', tau_req(4:6));
fprintf('  必要な床反力 [N]: [%7.4f, %7.4f]\n', f_ground);

%% 検証：計算された関節トルクで順動力学を実行
[ddq_verify, lambda_verify] = forward_dynamics_constrained(q, dq, tau_req, model);

fprintf('\n=== 検証結果 ===\n');
fprintf('実際の加速度:\n');
fprintf('  ベース線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq_verify(1:3));
fprintf('目標との誤差: %7.4f\n', norm(ddq_verify(1:3) - ddq_target(1:3)));
fprintf('実際の床反力 [N]: [%7.4f, %7.4f]\n', lambda_verify);
fprintf('床反力の誤差: %7.4f\n', norm(lambda_verify - f_ground));