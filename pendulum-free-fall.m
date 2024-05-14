%% Free-fall of the pendulum - without damping and l = 1
clear
clc
close all

% Parameters
m = 0.2; % Mass
g = 9.81; % Gravity

% General variables
h = 0.01;  % Step size
time = 0:h:5;  % Range of time
theta = zeros(size(time));  % Theta
w = zeros(size(time));  % Angular velocity
wa = zeros(size(time));  % Angular acceleration

% Initial Values
theta(1) = pi/4; % Initial theta
w(1) = 0;  % Initial angular velocity
wa(1) = 0; % Initial angular acceleration

% Euler's method to numerically integrate
for i=1:length(w)-1
    theta(i+1) = theta(i) + h*w(i); % Next theta
    w(i+1) = w(i) + h*wa(i); % Next velocity
    wa(i+1) = -g*sin(theta(i+1)); % Next acceleration
end

% ODE Method
figure(1);
set(gcf, 'Position',  [800, 100, 520, 500])
f = @(t,x) [x(2);-g*sin(x(1))];
init = [theta(1), w(1)];
[t,x] = ode45(f,[0 5],init);

% Theta Plots
subplot(2,1,1);
plot(t,x(:,1)); % Plotting ODE result
title('ODE Theta Result');
xlabel('Time')
ylabel('Theta (rad)')
grid on
subplot(2,1,2);
plot(time,theta); % Plotting Euler result
xlabel('Time')
ylabel('Theta (rad)')
title('Euler Theta Result');
grid on

%% Animation
figure(2)
a = axes;
set(gcf, 'Position',  [100, 100, 520, 500])
grid on;

for i=1:length(w)-1
    P = [sin(theta(i)) -cos(theta(i))];

    plot(a,[0,P(1)], [0,P(2)], 'Linewidth', 2, 'color', 'r'); % rod
    hold on
    plot(a,P(1),P(2),'.', 'MarkerSize',30); % ball    
    hold off

    title(a, 'Animation') 
    axis(a, 'equal')
    axis(a, [-1.5 1.5 -1.5 0.5]);
    grid on;
    pause(0.01);
end