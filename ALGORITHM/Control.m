function command = Control(gains,triad,est,w_des,meas)

addpath('ALGORITHM/transforms/')
addpath('ALGORITHM/models/')

w_error = meas.w - w_des;
MRP_est = est.mkp(1:3);
MRP_error = MRP_est - triad.MRP_des;

command = gains.Kd * w_error + gains.Kp * MRP_error;

end