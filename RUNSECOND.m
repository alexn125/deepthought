clear all
close all
% clc

t = tcpclient("127.0.0.1",10002);

gyroindex = 25;
whlindex = 21;
gpsindex = 24;
startrackindex = 20;
trueattindex = 16;
avindex = 16;
sunindex = 16;

itsmax = 101;
its = 1;
gnccount = 1;
RETURNMSG = false;

while its <= itsmax
    %% Read the socket
    if its > 1
        readline(t); %' '
        readline(t); % [EOF]
    end
    timestr = readline(t);
    readline(t); % ParmLoadEnabled
    readline(t); % ParmDumpEnabled
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
    % Reaction wheels
    whl1str = readline(t);
    whl2str = readline(t);
    whl3str = readline(t);
    % Accelerometers
    acc1str = readline(t);
    acc2str = readline(t);
    acc3str = readline(t);
    readline(t); % [EOF]

    %% Process socket msg
    if its == 1 || gnccount == 11
        gnccount = 1;

        w_meas = [str2double(gyro1str{1}(gyroindex:end)); str2double(gyro2str{1}(gyroindex:end)); str2double(gyro3str{1}(gyroindex:end))];

        rxnwheelmom = [str2double(whl1str{1}(whlindex:end)); str2double(whl2str{1}(whlindex:end)); str2double(whl3str{1}(whlindex:end))];

        gps_inter = replace(GPSposstr,'e+0','e');
        gps_inter2 = replace(gps_inter,'e-0','e-');
        gps_pos = str2num(gps_inter2{1}(gpsindex:end))';

        st_inter = replace(st_str,'e+0','e');
        st_inter2 = replace(st_inter,'e-0','e-');
        st_att = str2num(st_inter2{1}(startrackindex:end));

        true_attinter = replace(attstrtrue,'e+0','e');
        true_attinter2 = replace(true_attinter,'e-0','e-');
        at_true = str2num(true_attinter2{1}(trueattindex:end));
        
        w1 = replace(angvelstrtrue,'e+0','e');
        w2 = replace(w1,'e-0','e-');
        w_true = str2num(w2{1}(avindex:end));

        s1 = replace(sunpntstr_in,'e+0','e');
        s2 = replace(s1,'e-0','e-');
        sunpnt_true = str2num(s2{1}(sunindex:end));

        if t.NumBytesAvailable <= 1
            RETURNMSG = true;
        end
        %% Do GNC

        %% Process commands
    end
    %% Send commands

    write(t,'Ack')

    if RETURNMSG == true
        % send command
        % disp(its)
    end

    writeline(t,'[EOF]')

    its = its + 1;
    gnccount = gnccount + 1;
    RETURNMSG = false;
end