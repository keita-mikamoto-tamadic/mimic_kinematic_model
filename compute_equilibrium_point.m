function [q0, dq0, tau0] = compute_equilibrium_point(model, config)
% 平衡点の計算
% 脚車輪型倒立振子の直立静止状態での平衡点を求める
%
% 入力:
%   model  : ロボットモデル
%   config : 平衡点の設定（オプション）
%
% 出力:
%   q0   : 平衡点での一般化座標 [12x1]
%   dq0  : 平衡点での一般化速度 [12x1] (= 0)
%   tau0 : 平衡点での関節トルク [6x1]

if nargin < 2
    config = struct();
end

% デフォルト設定
default_config = struct(...
    'base_height', 0.25, ...          % ベース高さ [m]
    'base_orientation', [0; 0; 0], ... % ベース姿勢 [rad]
    'hip_angle', deg2rad(20), ...     % 股関節角度 [rad]
    'knee_angle', deg2rad(-40), ...   % 膝関節角度 [rad]
    'wheel_angle', 0 ...              % ホイール角度 [rad]
);

% 設定のマージ
config = merge_config(default_config, config);

% 平衡点での一般化座標
q0 = zeros(12, 1);

% ベース位置・姿勢
q0(1:3) = [0; 0; config.base_height];     % ベース位置
q0(4:6) = config.base_orientation;        % ベース姿勢

% 右脚関節角度
q0(7) = config.hip_angle;                 % 右脚股関節
q0(8) = config.knee_angle;                % 右脚膝関節
q0(9) = config.wheel_angle;               % 右ホイール

% 左脚関節角度
q0(10) = config.hip_angle;                % 左脚股関節
q0(11) = config.knee_angle;               % 左脚膝関節
q0(12) = config.wheel_angle;              % 左ホイール

% 平衡点での一般化速度（静止状態）
dq0 = zeros(12, 1);

% 両輪が床面に接触するように高さを調整
kin = compute_kinematics(q0, model);
wheel_radius = 0.05;  % タイヤ半径

% 右輪と左輪の接触点高さを確認
wheel_height_R = kin.p_links_R{3}(3);
wheel_height_L = kin.p_links_L{3}(3);

% ベース高さの調整（両輪が床面に接触するように）
target_wheel_height = wheel_radius;
current_wheel_height = min(wheel_height_R, wheel_height_L);
height_adjustment = target_wheel_height - current_wheel_height;
q0(3) = q0(3) + height_adjustment;

% 平衡点での重力補償トルク
[tau0, f_ground] = compute_gravity_compensation(q0, model);

% 結果の表示
fprintf('=== 平衡点の計算結果 ===\n');
fprintf('ベース位置 [m]: [%7.4f, %7.4f, %7.4f]\n', q0(1:3));
fprintf('ベース姿勢 [deg]: [%7.2f, %7.2f, %7.2f]\n', rad2deg(q0(4:6)));
fprintf('右脚関節 [deg]: [%7.2f, %7.2f, %7.2f]\n', rad2deg(q0(7:9)));
fprintf('左脚関節 [deg]: [%7.2f, %7.2f, %7.2f]\n', rad2deg(q0(10:12)));
fprintf('重力補償トルク [Nm]:\n');
fprintf('  右脚: [%7.4f, %7.4f, %7.4f]\n', tau0(1:3));
fprintf('  左脚: [%7.4f, %7.4f, %7.4f]\n', tau0(4:6));
fprintf('床反力 [N]: [%7.4f, %7.4f] (合計: %7.4f)\n', f_ground, sum(f_ground));

% 拘束条件の確認
[phi, ~] = compute_constraints(q0, dq0, model);
fprintf('拘束条件の確認 phi: [%7.4f, %7.4f]\n', phi);
if max(abs(phi)) > 1e-3
    warning('拘束条件が満たされていません。平衡点の調整が必要です。');
end

end

function merged = merge_config(default_config, user_config)
% 設定のマージ
merged = default_config;
fields = fieldnames(user_config);
for i = 1:length(fields)
    merged.(fields{i}) = user_config.(fields{i});
end
end