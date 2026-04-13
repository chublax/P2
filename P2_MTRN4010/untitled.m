% Script A2: Linear discrete time simulation of inverted pendulum
% Run after z5488191.m is available on the MATLAB path

clear;
clc;

M = 3.22;
m = 0.127;
l = 0.1778;
g = 9.81;
eta_m = 0.80;
Kt = 0.36;
rmp = 1.11e-2;
T = 0.005;

A = [0 0 1 0;
     0 0 0 1;
     0 m*g/M 0 0;
     0 (M+m)*g/(M*l) 0 0];

B = [0;
     0;
     eta_m*Kt/(M*rmp);
     eta_m*Kt/(M*l*rmp)];

C = eye(4);
D = zeros(4,1);

K = z5488191(A, B, C, D, T);

sys_d = c2d(ss(A, B, C, D), T, 'zoh');
[Ad, Bd, ~, ~] = ssdata(sys_d);

xd = 0.0;                    % desired cart position [m]
x  = [0; deg2rad(-5); 0; 0]; % [x; theta; xdot; thetadot]

tspan = 0:T:5;
N = length(tspan);

X = zeros(4, N);
X(:,1) = x;

for k = 1:N-1
    u = -K * X(:,k) + K(1) * xd;
    X(:,k+1) = Ad * X(:,k) + Bd * u;
end

figure;

subplot(2,1,1);
plot(tspan, rad2deg(X(2,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('θ (deg)');
title('Pendulum Angle');
grid on;

subplot(2,1,2);
plot(tspan, X(1,:)*100, 'r--', 'LineWidth', 1.5);
yline(xd*100, 'k:', 'x_d');
xlabel('Time (s)');
ylabel('Cart position (cm)');
title('Cart Position');
grid on;
