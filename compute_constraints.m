function [phi, J_phi, J_phi_dot] = compute_constraints(q, dq, model)
% 床面接触拘束条件の計算
% 両輪が床面（Z=0）に接触している条件

% 運動学計算
kin = compute_kinematics(q, model);

% 右輪と左輪の接触点位置
p_wheel_R = kin.p_links_R{3};  % 右輪中心位置
p_wheel_L = kin.p_links_L{3};  % 左輪中心位置

% 輪の半径（パラメータファイルから取得）
wheel_radius = model.wheel_radius;

% 拘束条件：両輪の接触点がZ=0平面に接触
phi = [
    p_wheel_R(3) - wheel_radius;  % 右輪接触点のZ座標 = 0
    p_wheel_L(3) - wheel_radius   % 左輪接触点のZ座標 = 0
];

% 拘束条件のヤコビアン計算
if nargout > 1
    % 数値微分によるヤコビアン計算
    delta = 1e-6;
    n = length(q);
    J_phi = zeros(2, n);
    
    for i = 1:n
        q_plus = q;
        q_plus(i) = q(i) + delta;
        kin_plus = compute_kinematics(q_plus, model);
        
        p_wheel_R_plus = kin_plus.p_links_R{3};
        p_wheel_L_plus = kin_plus.p_links_L{3};
        
        phi_plus = [
            p_wheel_R_plus(3) - wheel_radius;
            p_wheel_L_plus(3) - wheel_radius
        ];
        
        J_phi(:, i) = (phi_plus - phi) / delta;
    end
end

% 拘束条件の時間微分のヤコビアン
if nargout > 2
    % J_phi_dot = d/dt(J_phi)の計算
    % 簡単のため、ゼロ行列で近似（準静的解析）
    J_phi_dot = zeros(size(J_phi));
end

end