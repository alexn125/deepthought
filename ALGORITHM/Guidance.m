function att_des = Guidance(gps_pos,sunpnt_true,body1,body2)

addpath('ALGORITHM/transforms/')
addpath('ALGORITHM/models/')

ROT_MAT = TRIAD(-1*gps_pos,sunpnt_true,body1,body2);

out = DCM_MRP('DCMtoMRP',ROT_MAT);
quat_des = out(1:4);
MRP_des = out(5:7);

att_des = [quat_des;MRP_des];

end