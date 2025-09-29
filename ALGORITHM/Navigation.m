function [mkp,Pkp] = Navigation(est,meas,noise,sim_time,dt)

addpath('ALGORITHM/transforms/')
addpath('ALGORITHM/models/')
%% parse structures
mkm1 = est.mkm1;
Pkm1 = est.Pkm1;
Hk = est.H;

R = noise.R;
Q = noise.Q;

w_meas = meas.w;
att = meas.MRP;

%% prep for integration
pkm1 = mkm1(1:3);
bkm1 = mkm1(4:6);
n = height(mkm1);

int_inputs = [mkm1;Pkm1(:)];

w_hat = w_meas - bkm1; %estimated true angular velocity

B = (1-pkm1'*pkm1)*eye(3,3) + 2*skew(pkm1) + 2*(pkm1*pkm1');
F1 = (1/2)*(pkm1*w_hat' - w_hat*pkm1' - skew(w_hat) + (w_hat'*pkm1)*eye(3,3));
F_hat = [F1 (-1/4)*B; zeros(3,3) zeros(3,3)];
G_hat = [(-1/4)*B zeros(3,3);zeros(3,3) eye(3,3)];

%% integrate
[~,X] = ode45(@(t,X) dynamics_model(t,X,w_meas,noise.eta1,noise.eta2,F_hat,G_hat,Q,n),[sim_time sim_time+dt],int_inputs);

mkm = transpose(X(end,1:n));
Pkm = reshape(transpose(X(end,n+1:end)),n,n);
est.mkm = mkm;
est.Pkm = Pkm;

%% measurement update prep
pkm = mkm(1:3); % estimated MRPs at step k, before update
% bkm = mkm(4:6); % estimated gyro biases at step k, before update

if norm(pkm) > 1.0
    [mkm,Pkm] = shadowset(mkm,Pkm);
    % pkm = mkm(1:3);
    % bkm = mkm(4:6);
end

%% check attitude measurement for singularity
ykopt1 = att-Hk*mkm;
ykopt2 = (-att/(norm(att)^2))-Hk*mkm;

if norm(att)>(1/3)
    if norm(att)>norm(-att/(norm(att)^2))
        yk = ykopt2;
    else
        yk = ykopt1;
    end
else
    yk = ykopt1;
end

%% mean and covariance update
Kk = Pkm*Hk'*inv(Hk*Pkm*Hk' + R);
mkp = mkm + Kk*yk;
Pkp = (eye(6) - Kk*Hk)*Pkm;
est.mkp = mkp;
est.Pkp = Pkp;

%% set indices for next time
mkm1 = mkp;
Pkm1 = Pkp;
est.mkm1 = mkm1;
est.Pkm1 = Pkm1;

end