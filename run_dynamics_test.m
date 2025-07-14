clear; clc;

% パスの設定
addpath('./function_utill');

% モデル定義
model = model_definition_numeric();

% 初期姿勢の設定
q = zeros(12,1);
q(4:6) = deg2rad([0; 0; 80]);        % ボディ回転角度 [phi, theta, psi]
q(7) = deg2rad(38);                  % 右脚股関節
q(8) = -2*q(7);                      % 右脚膝関節
q(10) = deg2rad(38);                 % 左脚股関節
q(11) = -2*q(10);                    % 左脚膝関節

% 一般化速度の設定（フローティングベースロボット）
dq = zeros(12,1);
% ベース部分は自由度として扱う（外力なし）
% dq(1:6) = 0;  % ベース位置・姿勢の微分は0に初期化
% dq(7:12)は関節角速度（必要に応じて設定）

% 運動学計算と3D可視化
kin = compute_kinematics(q, model);
visualize_robot(kin, model);

%% 逆動力学テスト：コリオリ力＋重力項の計算
ddq0 = zeros(12,1);
[f_b0, tau0] = inverse_dynamics(q, dq, ddq0, model);

fprintf('\n=== 逆動力学結果 (C + G) ===\n');
fprintf('ベース力・モーメント [f_b0]:\n');
fprintf('  力  [N]: [%7.4f, %7.4f, %7.4f]\n', f_b0(1:3));
fprintf('  モーメント [Nm]: [%7.4f, %7.4f, %7.4f]\n', f_b0(4:6));
fprintf('関節トルク [tau0]:\n');
fprintf('  右脚 [Nm]: [%7.4f, %7.4f, %7.4f]\n', tau0(1:3));
fprintf('  左脚 [Nm]: [%7.4f, %7.4f, %7.4f]\n', tau0(4:6));

%% 順動力学テスト：関節トルク入力による加速度計算
% フローティングベースロボット：ベース部分への外力は0
tau_input = [0; 0; 0; 0; 0; 0];  % 関節トルク入力 [右脚3軸, 左脚3軸]
ddq = forward_dynamics(q, dq, tau_input, model);

fprintf('\n=== 順動力学結果 (ddq) - 自由落下状態 ===\n');
fprintf('ベース加速度:\n');
fprintf('  線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq(1:3));
fprintf('  角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq(4:6));
fprintf('関節加速度:\n');
fprintf('  右脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq(7:9));
fprintf('  左脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq(10:12));

%% 関節トルクありの場合のテスト
tau_input2 = [5; -5; 0.2; -0.2; 0.1; -0.1];  % 関節トルク入力
ddq2 = forward_dynamics(q, dq, tau_input2, model);

fprintf('\n=== 順動力学結果 (ddq) - 関節トルクあり ===\n');
fprintf('ベース加速度:\n');
fprintf('  線形 [m/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq2(1:3));
fprintf('  角 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq2(4:6));
fprintf('関節加速度:\n');
fprintf('  右脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq2(7:9));
fprintf('  左脚 [rad/s²]: [%7.4f, %7.4f, %7.4f]\n', ddq2(10:12));
