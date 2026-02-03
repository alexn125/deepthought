clear all
close all
clc

addpath('ALGORITHM')
addpath('RigidBodyKinematicsSchaubBook/Matlab/')
d2r = pi/180;
r2d = 180/pi;

period = 5573.647428028485; %orbital period

t = tcpclient("127.0.0.1",10002);

gyroindex = 25;
whlindex = 21;
gpsindex = 24;
startrackindex = 20;
trueattindex = 16;
avindex = 16;
sunindex = 16;

sim_length = 3600;
sim.rate = 1; %Hz

itsmax = sim_length*sim.rate + 1;
its = 1;
gnccount = 1;
RETURNMSG = false;

sim_time = 0.0;

% simplots = true;

%% initialize nav

initial_MRP_est = [0.1;0.3;-0.2];
initial_bias_est = [zeros(3,1)];
m0 = [initial_MRP_est;initial_bias_est];

GYROsd = 0.001*d2r; % gyro standard deviation rad/s
noise.eta1 = GYROsd*randn(3,1);
noise.eta2 = GYROsd*randn(3,1);

P0 = diag([0.1,0.1,0.1,0.1,0.1,0.1]);

est.mkm1 = m0;
est.Pkm1 = P0;
est.mkm = zeros(6,1);
est.Pkm = zeros(6,6);
est.mkp = zeros(6,1);
est.Pkp = zeros(6,6);
est.H = [eye(3,3) zeros(3,3)];


noise.R = 0.00003*eye(3,3);
noise.Q = 3e-9*eye(6,6);

cnt = 1;
nav.m_history = zeros(6,2*11);
nav.P_history = zeros(6,6,2*11);
nav.t_history = zeros(1,2*11);

nav.m_history(:,1) = m0;
nav.P_history(:,:,1) = P0;

stcount = 1;

%% Control gains

Kp = -1/100000;
Kd = -1/200000;
gains.Kp = Kp*eye(3,3);
gains.Kd = Kd*eye(3,3);

%% Some preallocation

stvalidstr = strings(itsmax,1);
stvec = zeros(sim_time+1,1);
triadhistory = zeros(4,sim_time+1);
commandhistory = zeros(3,sim_time+1);

%% GNC loop
while its <= itsmax
    %% Read the socket
    if its > 1
        readline(t); %' '
        readline(t); % [EOF]
    end
    datestr = readline(t);
    readline(t); % ParmLoadEnabled
    readline(t); % ParmDumpEnabled
    dtstr = readline(t);
    timestr = readline(t);
    angvelstrtrue = readline(t);
    attstrtrue = readline(t);
    sunpntstr_in = readline(t);
    % Gyro
    gyro1str = readline(t);
    gyro2str = readline(t);
    gyro3str = readline(t);
    % Star tracker
    stvalidstr(its) = readline(t); % valid
    st_str = readline(t);
    % GPS
    for i = 1:14
        if i == 5
            GPSposstr = readline(t);
        else
            readline(t);
        end
    end

    % Accelerometers
    acc1str = readline(t);
    acc2str = readline(t);
    acc3str = readline(t);

    % Reaction wheels
    whl1str = readline(t);
    whl2str = readline(t);
    whl3str = readline(t);

    readline(t); % [EOF]

    %% Process socket msg
    % if its == 1 || gnccount == 11
    gnccount = 1;

    if its == 1
        j2ktime_start = str2double(timestr{1}(17:end));
        dt = str2double(dtstr{1}(15:end));
    else
        sim_time = sim_time + dt;
        j2ktime = j2ktime_start + sim_time;
    end

    meas.w = [str2double(gyro1str{1}(gyroindex:end)); str2double(gyro2str{1}(gyroindex:end)); str2double(gyro3str{1}(gyroindex:end))];

    rxnwheelmom = [str2double(whl1str{1}(whlindex:end)); str2double(whl2str{1}(whlindex:end)); str2double(whl3str{1}(whlindex:end))];

    gps_inter = replace(GPSposstr,'e+0','e');
    gps_inter2 = replace(gps_inter,'e-0','e-');
    gps_pos_k = str2num(gps_inter2{1}(gpsindex:end))';

    % disp(its)
    % disp(st_str)

    st_inter = replace(st_str,'e+0','e');
    st_inter2 = replace(st_inter,'e-0','e-');
    st_att = str2num(st_inter2{1}(startrackindex:end))';

    true_attinter = replace(attstrtrue,'e+0','e');
    true_attinter2 = replace(true_attinter,'e-0','e-');
    at_true = str2num(true_attinter2{1}(trueattindex:end))';

    w1 = replace(angvelstrtrue,'e+0','e');
    w2 = replace(w1,'e-0','e-');
    w_true = str2num(w2{1}(avindex:end))';

    s1 = replace(sunpntstr_in,'e+0','e');
    s2 = replace(s1,'e-0','e-');
    sunpnt_true = str2num(s2{1}(sunindex:end))';

    if t.NumBytesAvailable <= 1
        RETURNMSG = true;
    end
    %% Do GNC

    meas.MRP = zeros(3,1); % attitude measurement
    for i = 1:3
        meas.MRP(i) = st_att(i)/(1+st_att(4));
    end

    meas.valid = false;
    if strcmp(stvalidstr(its),"SC[0].AC.ST[0].Valid = 1")
        meas.valid = true;
        stvec(stcount) = NaN;
    else
        stvec(stcount) = 0;
    end


    [~,~,mkm,Pkm,mkp,Pkp] = Navigation(est,meas,noise,sim_time,dt);

    est.mkm = mkm;
    est.Pkm = Pkm;
    est.mkp = mkp;
    est.Pkp = Pkp;

    %store nav outputs
    cnt = cnt + 1;
    nav.m_history(:,cnt) = est.mkm;
    nav.P_history(:,:,cnt) = est.Pkm;
    nav.t_history(:,cnt) = sim_time+dt;

    if its < itsmax
        cnt = cnt + 1;
        nav.m_history(:,cnt) = est.mkp;
        nav.P_history(:,:,cnt) = est.Pkp;
        nav.t_history(:,cnt) = sim_time+dt;
    end

    body1 = [1.0;0.0;0.0];
    body2 = [0.0;1.0;0.0];

    % disp(gps_pos/norm(gps_pos))

    [quat_des,MRP_des] = Guidance(gps_pos_k,sunpnt_true,body1,body2);

    triad.quat_des = quat_des;
    triad.MRP_des = MRP_des;

    % disp(quat_des)

    triadhistory(:,stcount) = triad.quat_des;
    % triad.MRP_des = zeros(3,1);

    if its == 1
        t_com = [0;0;0];
        gps_pos_km1 = gps_pos_k;
    else
        vel_est = (gps_pos_k - gps_pos_km1)/sim.rate;
        vel_est = vel_est/norm(vel_est);

        gps_pos_k = gps_pos_k/norm(gps_pos_k);

        hhat = cross(gps_pos_k,vel_est);

        w_des_inertial = ((2*pi)/period)*hhat;
        w_des = DCM_MRP('MRPtoDCM',est.mkp(1:3))'*w_des_inertial;

        t_com = Control(gains,triad,est,w_des,meas);

        gps_pos_km1 = gps_pos_k;
    end

    commandhistory(:,stcount) = t_com;

    %% set navigation indices for next time
    mkm1 = mkp;
    Pkm1 = Pkp;
    est.mkm1 = mkm1;
    est.Pkm1 = Pkm1;

    %% Process commands

    t0str = append('SC[0].AC.Whl[0].Tcmd = ',mat2str(t_com(1)));
    t1str = append('SC[0].AC.Whl[1].Tcmd = ',mat2str(t_com(2)));
    t2str = append('SC[0].AC.Whl[2].Tcmd = ',mat2str(t_com(3)));
    RETURNMSG = true;

    %% Index iteration
    stcount = stcount + 1;

    % end
    %% Send commands

    % writeline(t,'Ack')
    % disp(its)
    if RETURNMSG == true
        writeline(t,t0str)
        writeline(t,t1str)
        writeline(t,t2str)
    end
    writeline(t,'Ack')
    writeline(t,'[EOF]')
    % disp(t)
    its = its + 1;
    gnccount = gnccount + 1;
    RETURNMSG = false;
end

addpath("GNCout/SIM_ALL_STUFF/")
clear t
load("Missions/AlexResearch42/sim_results/qbn.42")
load("Missions/AlexResearch42/sim_results/wbn.42")
save('GNCout/SIM_ALL_STUFF/results.mat')

