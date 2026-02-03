close all
clear all
clc

its = 1;
itsmax = 10;
t = tcpclient("127.0.0.1",10002);
while its < itsmax
    disp(its)
    disp(t.NumBytesAvailable)
    steve = read(t,t.NumBytesAvailable,"char");
    disp(t.NumBytesAvailable)
    l0 = 'Ack';
    l1 = 'SC[0].AC.Whl[0].Tcmd = 0.0001';
    l2 = 'SC[0].AC.Whl[1].Tcmd = 0.0001';
    l3 = 'SC[0].AC.Whl[2].Tcmd = 0.0001';
    l4 = '[EOF]';
    
    write(t,"[EOF]","char")
    disp(t.NumBytesWritten)
    
    its = its + 1;
    b = 1;
    disp('-----------------------------')
end