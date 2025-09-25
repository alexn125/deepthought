clear all
close all
clc

t = tcpclient("127.0.0.1",10002);

gyroindex = 25;
whlindex = 21;
gpsindex = 24;

itsmax = 100;
its = 0;
MSGDONE = false;

while its<=itsmax

    %% Read the socket
    if its > 0
        readline(t); %' '
        readline(t); % [EOF]
    end
    timestr = readline(t);
    readline(t); % ParmLoadEnabled
    readline(t); % ParmDumpEnabled
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
    
    if t.NumBytesAvailable <= 1
        MSGDONE = true;
    end
    %% Process socket msg

    w_actual = [str2double(gyro1str{1}(gyroindex:end)); str2double(gyro2str{1}(gyroindex:end)); str2double(gyro3str{1}(gyroindex:end))];
    rxnwheelmom = [str2double(whl1str{1}(whlindex:end)); str2double(whl2str{1}(whlindex:end)); str2double(whl3str{1}(whlindex:end))];
    
    gps_inter = replace(GPSposstr,'e+0','e');
    gps_inter2 = replace(gps_inter,'e-0','e-');
    gps_pos = str2num(gps_inter2{1}(gpsindex:end))';
    
    st_inter = replace(st_str,'e+0','e');
    st_inter2 = replace(st_inter,'e-0','e-');

    st_att = str2num(st_inter2{1}(20:end));
    %% Do GNC

    %% Process commands

    %% Send commands
    if MSGDONE == true
        write(t,'Ack')
        writeline(t,'[EOF]')
        disp(its)
    end

    its = its + 1;
    MSGDONE = false;
end