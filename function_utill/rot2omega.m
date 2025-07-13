function omega = rot2omega(R)
% ROT2OMEGA  回転行列から軸ベクトル×角度を求める（ベクトル形式）
% 入力:
%   R: 3x3 回転行列
% 出力:
%   omega: 3x1 ベクトル, 回転軸*回転角（ラジアン）

    theta = acos( (trace(R) - 1) / 2 );

    if abs(theta) < 1e-8
        omega = [0; 0; 0];
        return;
    end

    omega_hat = (R - R') / (2 * sin(theta));
    omega = vee(omega_hat) * theta;
end

function v = vee(S)
% VEE: スキュー行列をベクトルに戻す
    v = [ S(3,2); S(1,3); S(2,1) ];
end