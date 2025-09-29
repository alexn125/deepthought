function [x_s,P_s] = shadowset(x,P)

p = x(1:3);
b = x(4:6);

pp = norm(p);

S = 2*(pp^(-4))*(p*p') - (pp^(-2))*eye(3,3);
L = [S zeros(3,3);zeros(3,3) eye(3,3)];

% P1 = P(1:3,1:3);
% P2 = P(1:3,4:6);
% P3 = P(4:6,1:3)';
% P4 = P(4:6,4:6);

x_s = [-p/(p'*p);b];
P_s = L*P*L';

end