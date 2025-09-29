function xdot = dynamics_model(~,X,w_meas,eta1,eta2,F,G,Q,n)

p = X(1:3); %MRP
b = X(4:6); %biases
% x = X(1:n);
P = reshape(X(n+1:end), n, n);

B = (1-p'*p)*eye(3,3) + 2*skew(p) + 2*(p*p');

% integrate mean

f = [(1/4)*B*(w_meas-b);zeros(3,1)];
g = [(-1/4)*B*eta1;eta2];

mdot = f+g;

% inner = 0.5*(1-p'*p)*eye(3) + skew(p) + p*p';
% f = 0.5*inner*(w_meas-b);
% g = -0.5*inner*eta1;
% pdot = f+g;
% bdot = eta2;

%integrate covariance
Pdot = F*P + P*F' + G*0.01*G' + Q;

% Pdot = F*P + P*F' + G*Q*G';

xdot = [mdot;Pdot(:)];

end