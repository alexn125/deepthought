% plotting script

clc
close all
clear all

% load('GNCout/SIM_ALL_STUFF/results.mat') % < --- most recent sim
% load('GNCout/SIM_ALL_STUFF/FULLresults.mat') % < --- sim where everything works (2/13)
load('GNCout/SIM_ALL_STUFF/NAVONLYresults.mat') % < --- nav only sim (2/20)
addpath('RigidBodyKinematicsSchaubBook/Matlab/')
addpath('ALGORITHM/transforms')

rad2deg = 180/pi;
%% - NAV plots

pvec = nav.m_history(1:3,:);
bvec = nav.m_history(4:6,:);
tt = nav.t_history;

time_vec = tt(1:2:end);

Pvec = zeros(6,cnt);
for i = 1:6
    Pvec(i,:) = sqrt(nav.P_history(i,i,:));
end

% load("Missions/AlexResearch42/sim_results/qbn.42")
len = size(qbn);
MRPtruth = zeros(len(1),3);
MRPtriad = zeros(3,length(time_vec));
addpath("ALGORITHM/transforms/")

for i = 1:len(1)
    conv = (1/(1+qbn(i,4)))*[qbn(i,1); qbn(i,2); qbn(i,3)];
    if norm(conv) >= 1.0
        MRPtruth(i,:) = conv*(-1./(conv'*conv));
    else
        MRPtruth(i,:) = conv;
    end
    if i < len(1)
        conv2 = (1/(1+triadhistory(1,i)))*[triadhistory(2,i); triadhistory(3,i); triadhistory(4,i)];
        if norm(conv2) >= 1.0
            MRPtriad(:,i) = conv2*(-1./(conv2'*conv2));
        else
            MRPtriad(:,i) = conv2;
        end    
    end
end
    

% figure
% t = tiledlayout(4,1);
% title(t,'Star Tracker Measured Attitude, Quaternions')
% nexttile
% plot(time_vec, startrackhistory(1,:))
% ylabel('q1')
% nexttile
% plot(time_vec, startrackhistory(2,:))
% ylabel('q2')
% nexttile
% plot(time_vec, startrackhistory(3,:))
% ylabel('q3')
% nexttile
% plot(time_vec, startrackhistory(4,:))
% ylabel('Scalar part')

figure
t = tiledlayout(3,1);
title(t,'Truth vs. Estimated Attitude (MRP)')
nexttile
plot(tt,pvec(1,:))
hold on
plot(0:len(1)-1,MRPtruth(:,1)')
ylabel('MRP 1')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(2,:))
hold on
plot(0:len(1)-1,MRPtruth(:,2)')
ylabel('MRP 2')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(3,:))
hold on
plot(0:len(1)-1,MRPtruth(:,3)')
ylabel('MRP 3')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on

figure
t = tiledlayout(3,1);
title(t,'Truth vs. Estimated vs. Desired Attitude (MRP)')
nexttile
plot(tt,pvec(1,:))
hold on
plot(0:len(1)-1,MRPtruth(:,1)',time_vec,MRPtriad(1,:))
ylabel('MRP 1')
xlabel('Time (s)')
legend('Estimate','Truth','Triad Output','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(2,:))
hold on
plot(0:len(1)-1,MRPtruth(:,2)',time_vec,MRPtriad(2,:))
ylabel('MRP 2')
xlabel('Time (s)')
legend('Estimate','Truth','Triad Output','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(3,:))
hold on
plot(0:len(1)-1,MRPtruth(:,3)',time_vec,MRPtriad(3,:))
ylabel('MRP 3')
xlabel('Time (s)')
legend('Estimate','Truth','Triad Output','Location','best')
hold off
grid on

addpath('GNCout/')
% saveas(gcf,'GNCout/nav_att.png')

evec = zeros(3,length(time_vec));

for i = 1:length(time_vec)
    % evec(:,i) = MRPtruth(i,:)' - pvec(:,2*i-1);
    evec(:,i) = subMRP(MRPtruth(i,:)',pvec(:,2*i-1));
end

figure
t = tiledlayout(3,1);
title(t,'Attitude Error and 3 sigma covariance (MRP)')
nexttile
plot(time_vec,evec(1,:))
hold on
plot(tt,3*Pvec(1,:),'r',tt,-3*Pvec(1,:),'r')
legend('MRP 1','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
xlabel('Time (s)')
ylabel('MRP 1')
hold off
grid on
nexttile
plot(time_vec,evec(2,:))
hold on
plot(tt,3*Pvec(2,:),'r',tt,-3*Pvec(2,:),'r')
legend('MRP 2','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
xlabel('Time (s)')
ylabel('MRP 2')
hold off
grid on
nexttile
plot(time_vec,evec(3,:))
hold on
plot(tt,3*Pvec(3,:),'r',tt,-3*Pvec(3,:),'r')
legend('MRP 3','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
xlabel('Time (s)')
ylabel('MRP 3')
hold off
grid on
% saveas(gcf,'GNCout/nav_errcovar.png')

figure
t = tiledlayout(3,1);
title(t,'Attitude Estimation Error (MRP) and Star Tracker Measurement Validity')
nexttile
plot(time_vec,evec(1,:))
% ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 1 Error','ST Invalid','Location','best')
ylabel('MRP 1')
xlabel('Time (s)')
hold off
grid on
nexttile
plot(time_vec,evec(2,:))
% ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 2 Error','ST Invalid','Location','best')
ylabel('MRP 2')
xlabel('Time (s)')
hold off
grid on
nexttile
plot(time_vec,evec(3,:))
% ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 3 Error','ST Invalid','Location','best')
ylabel('MRP 3')
xlabel('Time (s)')
hold off
grid on
% saveas(gcf,'GNCout/nav_errSTvalidity.png')

figure
t = tiledlayout(3,1);
title(t,'Estimated gyroscope bias (rad/s)')
nexttile
plot(tt,bvec(1,:))
xlabel('Time (s)')
ylabel('x')
grid on
nexttile
plot(tt,bvec(2,:))
xlabel('Time (s)')
ylabel('y')
grid on
nexttile
plot(tt,bvec(3,:))
xlabel('Time (s)')
ylabel('z')
grid on

% figure
% t = tiledlayout(3,1);
% title(t,'Estimated vs True Gyro Bias, deg/hr')
% nexttile
% plot(tt,bvec(1,:)*r2d*3600)
% hold on
% yline(-1)
% hold off
% legend('Estimated','Truth')
% grid on
% nexttile
% plot(tt,bvec(2,:)*r2d*3600)
% hold on
% yline(-2)
% hold off
% legend('Estimated','Truth')
% grid on
% nexttile
% plot(tt,bvec(3,:)*r2d*3600)
% hold on
% yline(-3)
% hold off
% legend('Estimated','Truth')
% grid on

% load("Missions/AlexResearch42/sim_results/wbn.42")
len = size(wbn);

figure
t = tiledlayout(3,1);
title(t,"Truth and measured angular velocity (deg/s)")
nexttile
plot(0:len(1)-1,wbn(:,1)'*rad2deg,time_vec,gyrohistory(1,:)*rad2deg)
legend('Truth','Measured')
xlabel('Time (s)')
ylabel('$\omega_x$','Interpreter','latex')
grid on
nexttile
plot(0:len(1)-1,wbn(:,2)'*rad2deg,time_vec,gyrohistory(2,:)*rad2deg)
legend('Truth','Measured')
xlabel('Time (s)')
ylabel('$\omega_y$','Interpreter','latex')
grid on
nexttile
plot(0:len(1)-1,wbn(:,3)'*rad2deg,time_vec,gyrohistory(3,:)*rad2deg)
legend('Truth','Measured')
xlabel('Time (s)')
ylabel('$\omega_z$','Interpreter','latex')
grid on

% figure
% t = tiledlayout(3,1);
% title(t,"Desired angular velocity from TRIAD (rad/s)")
% nexttile
% plot(time_vec,wdeshistory(1,:))
% xlabel('Time (s)')
% ylabel('$\omega_x$','Interpreter','latex')
% grid on
% nexttile
% plot(time_vec,wdeshistory(2,:))
% xlabel('Time (s)')
% ylabel('$\omega_y$','Interpreter','latex')
% grid on
% nexttile
% plot(time_vec,wdeshistory(3,:))
% xlabel('Time (s)')
% ylabel('$\omega_z$','Interpreter','latex')
% grid on

MRPtriad = zeros(3,length(time_vec));
for i = 1:length(time_vec)
    MRPtriad(:,i) = qtoMRP(triadhistory(:,i),1);
end

% figure
% t = tiledlayout(4,1);
% title(t, "Desired attitude from TRIAD method")
% nexttile
% plot(time_vec, triadhistory(2,:))
% ylabel('q1')
% xlabel('Time (s)')
% nexttile
% plot(time_vec, triadhistory(3,:))
% ylabel('q2')
% xlabel('Time (s)')
% nexttile
% plot(time_vec, triadhistory(4,:))
% ylabel('q3')
% xlabel('Time (s)')
% nexttile
% plot(time_vec, triadhistory(1,:))
% ylabel('Scalar part')
% xlabel('Time (s)')

figure
t = tiledlayout(3,1);
title(t, "Desired attitude from TRIAD method (MRP)")
nexttile
plot(time_vec, MRPtriad(1,:))
ylabel('MRP 1')
xlabel('Time (s)')
nexttile
plot(time_vec, MRPtriad(2,:))
ylabel('MRP 2')
xlabel('Time (s)')
nexttile
plot(time_vec, MRPtriad(3,:))
ylabel('MRP 3')
xlabel('Time (s)')

figure
t = tiledlayout(3,1);
title(t, "Torque Commands over time (N-m)")
nexttile
plot(time_vec, commandhistory(1,:))
ylabel('u1')
xlabel('Time (s)')
nexttile
plot(time_vec, commandhistory(2,:))
ylabel('u2')
xlabel('Time (s)')
nexttile
plot(time_vec, commandhistory(3,:))
ylabel('u3')
xlabel('Time (s)')

figure
t = tiledlayout(3,1);
title(t, "MRP error (control) over time")
nexttile
plot(time_vec, MRPerrorhistory(1,:))
ylabel('MRP 1')
xlabel('Time (s)')
nexttile
plot(time_vec, MRPerrorhistory(2,:))
ylabel('MRP 2')
xlabel('Time (s)')
nexttile
plot(time_vec, MRPerrorhistory(3,:))
ylabel('MRP 3')
xlabel('Time (s)')

% figure
% t = tiledlayout(3,1);
% title(t, "Sun pointing unit vector (inertial)")
% nexttile
% plot(time_vec, sunhistory(1,:))
% ylabel('x')
% xlabel('Time (s)')
% nexttile
% plot(time_vec, sunhistory(2,:))
% xlabel('Time (s)')
% ylabel('y')
% nexttile
% plot(time_vec, sunhistory(3,:))
% xlabel('Time (s)')
% ylabel('z')
% 
% figure
% t = tiledlayout(3,1);
% title(t, "GPS measurements (meters)")
% nexttile
% plot(time_vec, GPShistory(1,:))
% xlabel('Time (s)')
% ylabel('x')
% nexttile
% plot(time_vec, GPShistory(2,:))
% xlabel('Time (s)')
% ylabel('y')
% nexttile
% plot(time_vec, GPShistory(3,:))
% xlabel('Time (s)')
% ylabel('z')

% figure
% t = tiledlayout(3,1);
% nexttile
% plot(time_vec,cterms.term1h(1,:),time_vec,cterms.term2h(1,:),time_vec,cterms.term3h(1,:))
% legend('MRP term','Angular velocity error term','Third term')
% nexttile
% plot(time_vec,cterms.term1h(2,:),time_vec,cterms.term2h(2,:),time_vec,cterms.term3h(2,:))
% legend('MRP term','Angular velocity error term','Third term')
% nexttile
% plot(time_vec,cterms.term1h(3,:),time_vec,cterms.term2h(3,:),time_vec,cterms.term3h(3,:))
% legend('MRP term','Angular velocity error term','Third term')