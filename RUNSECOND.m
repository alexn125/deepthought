clear all
close all
clc

addpath('ALGORITHM')

d2r = pi/180;
r2d = 180/pi;

t = tcpclient("127.0.0.1",10002);

gyroindex = 25;
whlindex = 21;
gpsindex = 24;
startrackindex = 20;
trueattindex = 16;
avindex = 16;
sunindex = 16;

itsmax = 3001;
its = 1;
gnccount = 1;
RETURNMSG = false;

sim_time = 0.0;

simplots = true;

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

%% Control gains

Kp = -1/20;
Kd = -1/10;
gains.Kp = Kp*eye(3,3);
gains.Kd = Kd*eye(3,3);

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
    readline(t); % valid
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
    if its == 1 || gnccount == 11
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
        gps_pos = str2num(gps_inter2{1}(gpsindex:end))';

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

        body1 = [0;1;0];
        body2 = [0;0;1];

        % disp(gps_pos/norm(gps_pos))

        % [~,MRP_des] = Guidance(gps_pos,sunpnt_true,body1,body2);

        % triad.quat_des = quat_des;
        % triad.MRP_des = MRP_des;

        % disp(quat_des)
        
        % triad.quat_des = zeros(4,1);
        % triad.MRP_des = zeros(3,1);
        
        % w_des = [0;0;0];

        % t_com = Control(gains,triad,est,w_des,meas);

        %% set navigation indices for next time
        mkm1 = mkp;
        Pkm1 = Pkp;
        est.mkm1 = mkm1;
        est.Pkm1 = Pkm1;
        
        %% Process commands
        
        % t0str = append('SC[0].AC.Whl[0].Tcmd = ',mat2str(t_com(1)/100));
        % t1str = append('SC[0].AC.Whl[1].Tcmd = ',mat2str(t_com(2)/100));
        % t2str = append('SC[0].AC.Whl[2].Tcmd = ',mat2str(t_com(3)/100));
        % RETURNMSG = true;
    end
    %% Send commands

    write(t,'Ack')

    if RETURNMSG == true
        % writeline(t,t0str)
        % writeline(t,t1str)
        % writeline(t,t2str)
    end

    writeline(t,'[EOF]')

    its = its + 1;
    gnccount = gnccount + 1;
    RETURNMSG = false;
end

if simplots == true
    pvec = nav.m_history(1:3,:);
    bvec = nav.m_history(4:6,:);
    tt = nav.t_history;
    Pvec = zeros(6,cnt);
    for i = 1:6
        Pvec(i,:) = sqrt(nav.P_history(i,i,:));
    end

    load("Missions/AlexResearch42/sim_results/qbn.42")
    len = size(qbn);
    MRPtruth = zeros(len(1),3);

    addpath("ALGORITHM/transforms/")

    for i = 1:len(1)
        MRPtruth(i,:) = (1/(1+qbn(i,4)))*[qbn(i,1) qbn(i,2) qbn(i,3)];
    end    

    figure
    t = tiledlayout(3,1);
    title(t,'Estimated (MRP)')
    nexttile
    plot(tt,pvec(1,:))
    hold on
    plot(1:len(1),MRPtruth(:,1)')
    hold off
    grid on
    nexttile
    plot(tt,pvec(2,:))
    hold on
    plot(1:len(1),MRPtruth(:,2)')
    hold off
    grid on
    nexttile
    plot(tt,pvec(3,:))
    hold on
    plot(1:len(1),MRPtruth(:,3)')
    hold off
    grid on

    figure
    t = tiledlayout(3,1);
    title(t,'Estimated bias')
    nexttile
    plot(tt,bvec(1,:))
    grid on
    nexttile
    plot(tt,bvec(2,:))
    grid on
    nexttile
    plot(tt,bvec(3,:))
    grid on
end

