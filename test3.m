clear all
close all
clc

q0 = [0.0, 0.198019801980198, 0.0, 0.98019801980198]';
% q0 = q0./norm(q0);
% s0 = [-0.3;-0.4;0.2];
s0 = [0;0.1;0];
w0 = [-0.2;0.2;0.2]*(pi/180);
% w0 = zeros(3,1);

dt = 0.1;
sim.end = 1000;
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
torquevec = zeros(3, length(tspan));
% qvec(:,1) = q0;
svec(:,1) = s0;
wvec(:,1) = w0;
torquevec(:,1) = u_applied_0;
 
qdes = [0;0;0;1];
wdes = [0;0;0];
u_applied = u_applied_0;

Kp = 0.1;
Kd = 0.3;

tnow = 0.0;

% Run the simulation using ode45
for i = 1:length(tspan)
    disp(i)

    % [~,xout] = ode45(@(t, x) attEOMS(t, x, I, u_applied),[tnow tnow+dt],xkm1);
    [~,xout] = ode45(@(t, x) MRPattEOMS(t, x, I, u_applied),[tnow tnow+dt],xkm1);
    % qk = transpose(xout(end,1:4));
    % wk = transpose(xout(end,5:7));
    MRPk = transpose(xout(end,1:3));
    wk = transpose(xout(end,4:6));

    if i ~= tspan(end)
        % qvec(:,i+1) = qk;
        svec(:,i+1) = MRPk;
        wvec(:,i+1) = wk;
    end

    % MRPk = (1/(1+qk(4)))*[qk(1);qk(2);qk(3)];
    s2 = MRPk'*MRPk;
    if s2>1.0
        MRPk = (-1*MRPk) / s2; % shadowset
    end    
    

    % qerror = qk - qdes;
    w_error = wk - wdes;
    % u_applied = -1*(Kp*(qerror(1:3)/qerror(4)) + Kd*w_error);
    u_applied = -1*(Kd*wk + Kp*MRPk);
    % u_applied = -1*(Kp*qk(1:3) + Kd*w_error);
    % u_applied = [0;0;0];

    % xkm1 = [qk;wk];
    xkm1 = [MRPk;wk];
    


    torquevec(:,i+1) = u_applied;
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
plot(tspan,svec(1,1:end-1))
nexttile
plot(tspan,svec(2,1:end-1))
nexttile
plot(tspan,svec(3,1:end-1))


figure
t = tiledlayout(3,1);
title(t,'angular velocity')
nexttile
plot(tspan,wvec(1,1:end-1))
nexttile
plot(tspan,wvec(2,1:end-1))
nexttile
plot(tspan,wvec(3,1:end-1))

figure
t = tiledlayout(3,1);
title(t,'applied torque')
nexttile
plot(tspan,torquevec(1,1:end-1))
nexttile
plot(tspan,torquevec(2,1:end-1))
nexttile
plot(tspan,torquevec(3,1:end-1))

function xdot = attEOMS(~,x,I,u_applied)
% vector first
E = x(1:3);
qs = x(4);

% scalar first
% E = x(2:4);
% qs = x(1);

w = x(5:7);

qsdot = (-1/2)*E'*w;
T = qs*eye(3,3) + skew(E);
Edot = (1/2)*T*w;

%vector first
qdot = [Edot;qsdot];
%scalar first
% qdot = [qsdot;Edot];

wdot = I\(-1*skew(w)*I*w + u_applied);

xdot = [qdot;wdot];

    function ax = skew(a)
        ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
    end

end

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

% function ax = skew(a)
%         ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
% end