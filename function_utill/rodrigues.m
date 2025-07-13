function R = rodrigues(a, theta)
% Rodriguesの回転公式
% a: 回転軸ベクトル（3x1、正規化推奨）
% theta: 回転角（ラジアン）

    a = a / norm(a);      % 念のため正規化
    ahat = hat(a);        % スキュー対称行列
    R = eye(3) + sin(theta)*ahat + (1 - cos(theta))*(ahat*ahat);
end