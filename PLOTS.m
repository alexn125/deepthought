% plotting script

clc
close all
clear all

load('GNCout/SIM_ALL_STUFF/results.mat')

%% - NAV plots

pvec = nav.m_history(1:3,:);
bvec = nav.m_history(4:6,:);
tt = nav.t_history;

time_vec = tt(1:2:end);

Pvec = zeros(6,cnt);
for i = 1:6
    Pvec(i,:) = sqrt(nav.P_history(i,i,:));
end

load("Missions/AlexResearch42/sim_results/qbn.42")
len = size(qbn);
MRPtruth = zeros(len(1),3);

addpath("ALGORITHM/transforms/")

for i = 1:len(1)
    conv = (1/(1+qbn(i,4)))*[qbn(i,1); qbn(i,2); qbn(i,3)];
    if norm(conv) >= 1.0
        MRPtruth(i,:) = conv*(-1./(conv'*conv));
    else
        MRPtruth(i,:) = conv;
    end
end

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

addpath('GNCout/')
saveas(gcf,'GNCout/nav_att.png')

evec = zeros(3,length(time_vec));

for i = 1:length(time_vec)
    evec(:,i) = MRPtruth(i,:)' - pvec(:,2*i-1);
end

figure
t = tiledlayout(3,1);
title(t,'Attitude Error (truth - estimate) and 3 sigma covariance (MRP)')
nexttile
plot(time_vec,evec(1,:))
hold on
plot(tt,3*Pvec(1,:),'r',tt,-3*Pvec(1,:),'r')
legend('MRP 1','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
legend()
hold off
grid on
nexttile
plot(time_vec,evec(2,:))
hold on
plot(tt,3*Pvec(2,:),'r',tt,-3*Pvec(2,:),'r')
legend('MRP 2','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
hold off
grid on
nexttile
plot(time_vec,evec(3,:))
hold on
plot(tt,3*Pvec(3,:),'r',tt,-3*Pvec(3,:),'r')
legend('MRP 3','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
hold off
grid on
saveas(gcf,'GNCout/nav_errcovar.png')

figure
t = tiledlayout(3,1);
title(t,'Attitude Estimation Error (MRP) and Star Tracker Measurement Validity')
nexttile
plot(time_vec,evec(1,:))
ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 1 Error','ST Invalid','Location','best')
hold off
grid on
nexttile
plot(time_vec,evec(2,:))
ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 2 Error','ST Invalid','Location','best')
hold off
grid on
nexttile
plot(time_vec,evec(3,:))
ylim([-0.1 0.1])
hold on
scatter(time_vec,stvec+0.05,5,'r','x')
legend('MRP 3 Error','ST Invalid','Location','best')
hold off
grid on
saveas(gcf,'GNCout/nav_errSTvalidity.png')

% figure
% t = tiledlayout(3,1);
% title(t,'Estimated bias')
% nexttile
% plot(tt,bvec(1,:))
% grid on
% nexttile
% plot(tt,bvec(2,:))
% grid on
% nexttile
% plot(tt,bvec(3,:))
% grid on

% load("Missions/AlexResearch42/sim_results/wbn.42")
% len = size(wbn);
%
% figure
% t = tiledlayout(3,1);
% title(t,"Truth angular velocity, rad/s")
% nexttile
% plot(0:len(1)-1,wbn(:,1)')
% grid on
% nexttile
% plot(0:len(1)-1,wbn(:,2)')
% grid on
% nexttile
% plot(0:len(1)-1,wbn(:,3)')
% grid on