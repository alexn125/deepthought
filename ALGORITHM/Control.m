function command = Control(gains,triad,est,w_des,w_des_dot,u_max,meas)

addpath('ALGORITHM/transforms/')
addpath('ALGORITHM/models/')
addpath('RigidBodyKinematicsSchaubBook/Matlab/')

w_error = meas.w - w_des;
MRP_est = est.mkp(1:3);
% MRP_error = MRP_est - triad.MRP_des;
MRP_error = subMRP(MRP_est,triad.MRP_des);

term1 = -1*gains.K*MRP_error;
term2 = -1*gains.P*w_error;
term3 = eye(3,3)*(w_des_dot - skew(meas.w)*w_des) + skew(w_des)*eye(3,3)*meas.w;

command = term1 + term2 + term3;

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