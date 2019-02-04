function dqdt = mr2d(t, q, k, m)
%  Motion equations in 2D repulsive potential
% ---------------------------------------------    
    dqdt = zeros(4,1); % it is VERY important statement
    % first dimension (x)
    dqdt(1) = q(2); % x-position equation
    dqdt(2) = k/m*q(1)/(q(1)^2 + q(3)^2)^2; % x-velocity equation
    % second dimension (y)
    dqdt(3) = q(4); % y-position equation
    dqdt(4) = k/m*q(3)/(q(1)^2 + q(3)^2)^2; % y-velocity equation
end

