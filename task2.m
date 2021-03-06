clear all; close all; clc;

syms x y k m t

V(x,y) = k/2*(x^2+y^2)

Fx = -diff(V,x)
Fy = -diff(V,y)

% Equations of motion
syms x(t) y(t)

eq1 = m*diff(x,t,t) == Fx(x,y)
eq2 = m*diff(y,t,t) == Fy(x,y)

% Solution
[xs(t) ys(t)] = dsolve(eq1, eq2);

% --------------------------------------------
% Numerical approach

% 1d oscillator diff(q,t,t)*m = -k*q (V = (k*x^2)/2)
tspan = [0 10*pi]; % time span

q0 = [-2; 0; 0; 1]; % pos_0; vel_0
% Solution
k=1; m=1;
[t, q] = ode45(@(t, q) ma2d(t, q, k, m), tspan, q0);

plot(t, q(:,1), '-o', t, q(:,3), '-.')
set(gca(),'FontSize',14)
xlim([min(t) max(t)]);
grid on
hold on
title('Solution with ODE45');
xlabel('Time t');
ylabel('Solution q');
legend('x','y')


% Phase Diagram
 figure; 
 plot(q(:,1), q(:,3),'-o');
 set(gca(),'FontSize',14)
 grid on
 xlabel('x-position');
 ylabel('y-position');
 title('Trajectory')
 axis equal;

% Energy conservation law check
n=numel(t);
tc=zeros(n-1, 1);

for i = 1:numel(t)-1
    tc(i) = (t(i)+t(i+1))/2;
end

xa = diff(q(:,2))./diff(t);
ya = diff(q(:,4))./diff(t);

q(:,5)=interp1(tc, xa, t, 'spline'); % x-acceleration
q(:,6)=interp1(tc, ya, t, 'spline'); % y-acceleration

for i=1:numel(t)
    x = q(i,1); vx = q(i,2); ax = q(i,5);
    y = q(i,3); vy = q(i,4); ay = q(i,6);
    dEdt(i) = m*(vx*ax + vy*ay) + k*(vx*x + vy*y);
    T(i) = m*(vx*vx + vy*vy)/2;
    W(i) = k*(x^2+y^2)/2;
end

for i=1:numel(t)-1
    dT(i)=(T(i+1)-T(i))/(t(i+1)-t(i));
    dW(i)=(W(i+1)-W(i))/(t(i+1)-t(i));
end

figure
plot(t,T,t,W,t,T+W)
set(gca(),'FontSize',14)
xlim([min(t) max(t)]);
grid on
xlabel('Time')
ylabel('Energy')
legend('T','W','T+W')

figure
plot(t, dEdt, tc, dT+dW)
set(gca(),'FontSize',14)
xlim([min(t) max(t)]);
grid on
xlabel('Time')
ylabel('dE/dt')
legend('Analytical','Numerical')