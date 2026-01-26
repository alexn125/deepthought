clear all
close all
clc

q0 = [0.0, 0.198019801980198, 0.0, 0.98019801980198]';
q0 = q0./norm(q0);
w0 = [-0.2;0.2;0.2]*(pi/180);

dt = 0.1;
sim.end = 1000;
tspan = 0:dt:sim.end;

I = [0.026 0 0;0 0.06 0; 0 0 0.085];
u_applied_0 = [0.0;0.0;0.0];

% Initialize state vector
x0 = [q0; w0];
xkm1 = x0;

timevec = zeros(1,length(tspan));
qvec = zeros(4,length(tspan));
wvec = zeros(3,length(tspan));
torquevec = zeros(3, length(tspan));
qvec(:,1) = q0;
wvec(:,1) = w0;
torquevec(:,1) = u_applied_0;
 
qdes = [0.5;0;0.5;0];
wdes = [0;0;0];
u_applied = u_applied_0;

Kp = 1;
Kd = 3;

tnow = 0.0;

% Run the simulation using ode45
for i = 1:length(tspan)
    disp(i)
    [~,xout] = ode45(@(t, x) attEOMS(t, x, I, u_applied),[tnow tnow+dt],xkm1);
    qk = transpose(xout(end,1:4));
    wk = transpose(xout(end,5:7));

    if i ~= tspan(end)
        qvec(:,i+1) = qk;
        wvec(:,i+1) = wk;
    end
     
    % qerror = qk - qdes;
    % w_error = wk - wdes;
    % u_applied = -1*(Kp*qerror + Kd*w_error);
    u_applied = skew(wk)*I*wk - Kd*wk - Kp*(qk(1:3)/qk(4));

    qkm1 = qk;
    wkm1 = wk;

    torquevec(:,i+1) = u_applied;
    tnow = tnow + dt;
end
% qout = transpose(xout(:,1:4));
% wout = transpose(xout(:,5:7));

figure
t = tiledlayout(4,1);
title(t,'attitude quaternions')
nexttile
plot(tspan,qvec(1,1:end-1))
nexttile
plot(tspan,qvec(2,1:end-1))
nexttile
plot(tspan,qvec(3,1:end-1))
nexttile
plot(tspan,qvec(4,1:end-1))

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

function ax = skew(a)
        ax = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
end