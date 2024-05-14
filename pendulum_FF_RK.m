%% Free-fall of the pendulum Runge Kutta vs Euler - Pendulum - without damping and l = 1
clear
clc
close all

% Parameters
m = 0.2; % Mass
g = 9.81; % Gravity
l = 1; % Length Rod

% General variables
h = 0.01;  % Step size
time = 0:h:20;  % Range of time

thetaE = zeros(size(time));  % Theta
wE = zeros(size(time));  % Angular velocity
waE = zeros(size(time));  % Angular acceleration

thetaRK = zeros(size(time));  % Theta
wRK = zeros(size(time));  % Angular velocity
waRK = zeros(size(time));  % Angular acceleration

% Initial values
thetaE(1) = pi/2; % Initial theta
wE(1) = 0;  % Initial angular velocity
waE(1) = 0; % Initial angular acceleration

thetaRK(1) = pi/4; % Initial theta
wRK(1) = 0;  % Initial angular velocity
waRK(1) = 0; % Initial angular acceleration

% Euler's method to numerically integrate
for i=1:length(wE)-1
    thetaE(i+1) = thetaE(i) + h*wE(i); % Next theta
    wE(i+1) = wE(i) + h*waE(i); % Next velocity
    waE(i+1) = (-g*sin(thetaE(i+1))/l); % Next acceleration
end

% 4 Order Runge-kutta
for i=1:(length(wRK)-1)

    % RK method
    k1theta = h*wRK(i);
    k1w =  h*(-g*sin(thetaRK(i))/l);
    
    k2theta = h*(wRK(i) + 0.5*k1w);
    k2w =  h*(-g*sin(thetaRK(i) + 0.5*k1theta)/l);
    
    k3theta = h*(wRK(i) + 0.5*k2w);
    k3w =  h*(-g*sin(thetaRK(i) + 0.5*k2theta)/l);

    k4theta = h*(wRK(i) + k3w);
    k4w =  h*(-g*sin(thetaRK(i) + k3theta)/l);  

    % Update theta and w
    thetaRK(i+1) = thetaRK(i) + (k1theta + 2*k2theta + 2*k3theta + k4theta)/6;
    wRK(i+1) = wRK(i) + (k1w + 2*k2w + 2*k3w + k4w)/6;

end

% Theta Plots
subplot(2,1,1);
plot(time,rad2deg(thetaE)); % Plotting Theta graph Euler
title('Theta Result (deg) EULER');
xlabel('Time')
ylabel('Theta')
grid on
subplot(2,1,2);
plot(time,rad2deg(thetaRK)); % Plotting Theta graph RK
title('Theta Result (deg) RUNGE KUTTA');
xlabel('Time')
ylabel('Theta')
grid on

%exportgraphics(gcf, 'plot.pdf', 'ContentType', 'vector');