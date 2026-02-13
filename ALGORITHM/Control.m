function [command,aux] = Control(gains,triad,est,w_des,w_des_dot,u_max,meas,J)

addpath('ALGORITHM/transforms/')
addpath('ALGORITHM/models/')
addpath('RigidBodyKinematicsSchaubBook/Matlab/')

w_error = meas.w - w_des;
MRP_est = est.mkp(1:3);
% MRP_error = MRP_est - triad.MRP_des;
MRP_error = subMRP(MRP_est,triad.MRP_des);

term1 = -1*gains.K*MRP_error;
term2 = -1*gains.P*w_error;
term3a = J*(w_des_dot - skew(meas.w)*w_des);
term3b = skew(w_des)*J*meas.w;

command = term1 + term2 + term3a + term3b;
aux = MRP_error;
% aux = term3a;

for j = 1:3
    if abs(command(j))>u_max
        if command(j)>0
            command(j) = u_max;
        else
            command(j) = -u_max;
        end
    end
end
% disp(command)
end