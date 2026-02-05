clear all
close all
clc

mu = 398600441800000.0;
a = 6794419.25330953;

addpath('ALGORITHM/transforms/')

period = 2*pi*sqrt((a^3)/mu);

w_dot_des = [0;0;0];

q0 = [0.0, 0.198019801980198, 0.0, 0.98019801980198]';
% q0 = q0./norm(q0);
% s0 = [-0.3;-0.4;0.2];
s0 = [0;0.1;0];
% w0 = [-0.2;0.2;0.2]*(pi/180);
w0 = zeros(3,1);

dt = 1;
sim.end = 50;
tspan = 0:dt:sim.end;

I = [0.026 0 0;0 0.06 0; 0 0 0.085];
% I = 10*eye(3,3);
u_applied_0 = [0.0;0.0;0.0];

% Initialize state vector
% x0 = [q0; w0];
x0 = [s0;w0];
xkm1 = x0;

timevec = zeros(1,length(tspan));
% qvec = zeros(4,length(tspan));
svec = zeros(3,length(tspan));
wvec = zeros(3,length(tspan));
w_inervec = zeros(3,length(tspan));
w_desinervec = zeros(3,length(tspan));
torquevec = zeros(3, length(tspan));
posvec = zeros(3,length(tspan));
velvec = zeros(3,length(tspan));
wdesvec = zeros(3,length(tspan));
MRPdesvec = zeros(3,length(tspan));
% qvec(:,1) = q0;
svec(:,1) = s0;
wvec(:,1) = w0;
torquevec(:,1) = u_applied_0;
 
qdes = [0;0;0;1];
wdes = [0;0;0];
u_applied = u_applied_0;

K = 0.01*eye(3,3);
P = 0.03*eye(3,3);

tnow = 0.0;

initpos = [5726667.125087506 -3656728.158222793 9195.914740745968]';
initvel = [2553.6769906688132 4009.614722441945 6005.3828595530185]';

posvec(:,1) = initpos;
velvec(:,1) = initvel;
wdesvec(:,1) = wdes;
rkm1 = initpos;
vkm1 = initvel;

angvec = zeros(1,length(tspan));

load("triadhistory.mat");

% Run the simulation using ode45
for i = 1:length(tspan)
    disp(i)
    [~,pvout] = ode45(@(t,x) PVEOMS(t,x,mu),[tnow tnow+dt],[rkm1;vkm1]);
    % [~,xout] = ode45(@(t, x) attEOMS(t, x, I, u_applied),[tnow tnow+dt],xkm1);
    [~,xout] = ode45(@(t, x) MRPattEOMS(t, x, I, u_applied),[tnow tnow+dt],xkm1);
    % qk = transpose(xout(end,1:4));
    % wk = transpose(xout(end,5:7));
    rk = transpose(pvout(end,1:3));
    vk = transpose(pvout(end,4:6));
    MRPk = transpose(xout(end,1:3));
    wk = transpose(xout(end,4:6));

    % MRPk = (1/(1+qk(4)))*[qk(1);qk(2);qk(3)];
    s2 = MRPk'*MRPk;
    if s2>1.0
        MRPk = (-1*MRPk) / s2; % shadowset
        s2 = MRPk'*MRPk;
    end    
    
    DCM = eye(3,3) + (8*skew(MRPk)*skew(MRPk) - 4*(1-s2)*skew(MRPk))/((1+s2)^2);
    
    quat_des = triadhistory(:,i);
    MRPdes = (1/(1+quat_des(1)))*[quat_des(2);quat_des(3);quat_des(4)];
    s2d = MRPdes'*MRPdes;
    if s2d>1.0
        MRPdes = (-1*MRPdes)/s2d;
    end    
    MRPdesvec(:,i) = MRPdes;
    % disp(DCM)

    rkhat = rk/(norm(rk));
    vkest = (rk - rkm1)/dt;
    vkhat = vkest/(norm(vkest));
    hhat = cross(rkhat,vkhat);

    % w_des = ((2*pi)/period)*transpose(DCM)*hhat;
    w_des = transpose(DCM)*((2*pi)/period)*hhat;
    % w_des = transpose(DCM)*cross(rk,vkest)/(norm(rk)*norm(rk));
    % control

    % qerror = qk - qdes;
    % w_error = wk - wdes;
    % u_applied = -1*(Kp*(qerror(1:3)/qerror(4)) + Kd*w_error);
    % u_applied = -1*(P*wk + K*MRPk);
    % u_applied = -1*(Kp*qk(1:3) + Kd*w_error);
    % u_applied = [0;0;0];
    
    % k_term = -1*K*MRPk;
    addpath('RigidBodyKinematicsSchaubBook/Matlab/')
    % k_term = -1*K*MRPk;
    k_term = -1*K*subMRP(MRPk,MRPdes);
    p_term = -1*P*(wk-w_des);
    % other_term = eye(3,3)*(w_dot_des - skew(wk)*w_des) + skew(w_des)*eye(3,3)*wk; % Schaub 432
    % u_applied = k_term + p_term + other_term;
    u_applied = k_term + p_term;
    u_max = 8/1000; % mN-m

    for j = 1:3
        if abs(u_applied(j))>u_max
            if u_applied(j)>0
                u_applied(j) = u_max; 
            else
                u_applied(j) = -u_max; 
            end
        end    
    end    
   
    % u_applied = -1*K*MRPk - P*(wk-w_des) + eye(3,3)*(w_dot_des - skew(wk)*w_des) + skew(w_des)*eye(3,3)*wk;
    
    % disp("K term")
    % disp(k_term)
    % disp("P term")
    % disp(p_term)
    % disp("other stuff")
    % disp(other_term)
    % disp('-----------------------------')

    if i < length(tspan)
        % qvec(:,i+1) = qk;
        svec(:,i+1) = MRPk;
        wvec(:,i+1) = wk;
        posvec(:,i+1) = rk;
        velvec(:,i+1) = vk;
        torquevec(:,i+1) = u_applied;
        wdesvec(:,i+1) = w_des;

        w_inervec(:,i+1) = DCM'*wk;
        w_desinervec(:,i+1) = ((2*pi)/period)*hhat;
    end

    % for next time

    bx_1 = DCM*[1;0;0];
    bx_1 = bx_1/norm(bx_1);
    ECI_x = rk/norm(rk);

    ang = acos(dot(bx_1,ECI_x));
    angvec(i) = ang;

    % xkm1 = [qk;wk];
    xkm1 = [MRPk;wk];
    rkm1 = rk;
    vkm1 = vk;
    tnow = tnow + dt;
end
% qout = transpose(xout(:,1:4));
% wout = transpose(xout(:,5:7));

% figure
% t = tiledlayout(4,1);
% title(t,'attitude quaternions')
% nexttile
% plot(tspan,qvec(1,1:end-1))
% nexttile
% plot(tspan,qvec(2,1:end-1))
% nexttile
% plot(tspan,qvec(3,1:end-1))
% nexttile
% plot(tspan,qvec(4,1:end-1))

figure
t = tiledlayout(3,1);
title(t,'MRPs')
nexttile
plot(tspan,svec(1,:),tspan,MRPdesvec(1,:))
legend('MRP truth','MRP from TRIAD')
nexttile
plot(tspan,svec(2,:),tspan,MRPdesvec(2,:))
legend('MRP truth','MRP from TRIAD')
nexttile
plot(tspan,svec(3,:),tspan,MRPdesvec(3,:))
legend('MRP truth','MRP from TRIAD')

figure
t = tiledlayout(3,1);
title(t,'angular velocity vs desired angular velocity')
nexttile
plot(tspan,wvec(1,:),tspan,wdesvec(1,:))
legend('true','desired')
nexttile
plot(tspan,wvec(2,:),tspan,wdesvec(2,:))
legend('true','desired')
nexttile
plot(tspan,wvec(3,:),tspan,wdesvec(3,:))
legend('true','desired')

figure
t = tiledlayout(3,1);
title(t,'angular velocity vs desired angular velocity, inertial frame')
nexttile
plot(tspan,w_inervec(1,:),tspan,w_desinervec(1,:))
legend('true','desired')
nexttile
plot(tspan,w_inervec(2,:),tspan,w_desinervec(2,:))
legend('true','desired')
nexttile
plot(tspan,w_inervec(3,:),tspan,w_desinervec(3,:))
legend('true','desired')

figure
t = tiledlayout(3,1);
title(t,'desired angular velocity error')
nexttile
plot(tspan,wvec(1,:)-wdesvec(1,:))
nexttile
plot(tspan,wvec(2,:)-wdesvec(2,:))
nexttile
plot(tspan,wvec(3,:)-wdesvec(3,:))

figure
t = tiledlayout(3,1);
title(t,'applied torque')
nexttile
plot(tspan,torquevec(1,:))
nexttile
plot(tspan,torquevec(2,:))
nexttile
plot(tspan,torquevec(3,:))

% figure
% title('Position')
% plot3(posvec(1,:),posvec(2,:),posvec(3,:))
% hold on
% scatter3(0,0,0)
% hold off

figure
plot(tspan,angvec*(180/pi))
title('Angle (Deg) between body x axis and ECI x axis')

% function xdot = attEOMS(~,x,I,u_applied)
% % vector first
% E = x(1:3);
% qs = x(4);
% 
% % scalar first
% % E = x(2:4);
% % qs = x(1);
% 
% w = x(5:7);
% 
% qsdot = (-1/2)*E'*w;
% T = qs*eye(3,3) + skew(E);
% Edot = (1/2)*T*w;
% 
% %vector first
% qdot = [Edot;qsdot];
% %scalar first
% % qdot = [qsdot;Edot];
% 
% wdot = I\(-1*skew(w)*I*w + u_applied);
% 
% xdot = [qdot;wdot];
% 
%     function ax = skew(a)
%         ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
%     end
% 
% end

function xdot = MRPattEOMS(~,x,I,u_applied)
s = x(1:3);
w = x(4:6);

s2 = s'*s;

B = (1-s2)*eye(3,3) + 2*skew(s) + 2*(s*s');
sdot = (1/4)*B*w;
wdot = I\(-1*skew(w)*I*w + u_applied);

xdot = [sdot;wdot];

    function ax = skew(a)
        ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
    end

end

function xdot = PVEOMS(~,x,mu)
    xdot = zeros(6,1);
    r = x(1:3);
    v = x(4:6);
    xdot(1:3) = v;
    xdot(4:6) = ((-1*mu)/((norm(r))^3))*r;
end

function ax = skew(a)
        ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
end