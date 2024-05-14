%% Control of the pendulum - PD+LQR
clear
clc
close all

% Physical Parameters
m = 0.5; % Mass
g = 9.81; % Gravity
l = 0.5; % Length Rod
b = 0.1; % Damping

% PD Control parameters
Kp = 15;
Kd = 3.0;
theta_goal = pi;

% LQR - Control parameters
A = [0, 1; g/l, -b];
B = [0; 1/(m*l*l)];
Q = diag([1, 1]);
R = 1;
K = lqr(A,B,Q,R);

% General variables
h = 0.01;  % Step size
time = 0:h:10;  % Range of time
theta = zeros(size(time));  % Theta
w = zeros(size(time));  % Angular velocity
wa = zeros(size(time));  % Angular acceleration
tau_vector = zeros(size(time));  % Actuation vector
tau_l = 1; % Torque limit

% Initial State
theta(1) = 0; % Initial theta
w(1) = 0;  % Initial angular velocity
wa(1) = 0; % Initial angular acceleration

% Euler's method to numerically integrate
for i=1:length(w)-1
    theta(i+1) = theta(i) + h*w(i); % Next theta
    w(i+1) = w(i) + h*wa(i); % Next velocity
    
    % Change torque direction if acceleration is too low
    if theta(i+1) == 0
        tau_vector(i)  = Kp*(theta_goal - theta(i+1)) - Kd*w(i+1);
    elseif theta(i+1) > 0
        tau_vector(i)  = Kp*(theta_goal - theta(i+1)) - Kd*w(i+1);
        if w(i+1) < 0.002 && abs(theta(i+1)) < 2.6
            tau_vector(i)  = -Kp*(theta_goal + theta(i+1)) - Kd*w(i+1);
        end        
    elseif theta(i+1) < 0
        tau_vector(i)  = -Kp*(theta_goal + theta(i+1)) - Kd*w(i+1);
        if w(i+1) > 0.002 && abs(theta(i+1)) < 2.6
            tau_vector(i)  = Kp*(theta_goal - theta(i+1)) - Kd*w(i+1);
        end
    end 
    
    % LQR - Activates when theta > 90º
    if abs(theta(i+1)) > pi/2
        tau_vector(i) = -K*[theta(i) - pi; w(i)];
    end

    % Torque limit
    if tau_vector(i) > tau_l
        tau_vector(i) = tau_l;    
    elseif tau_vector(i) < -tau_l
        tau_vector(i) = -tau_l;
    end   
    
    % Next acceleration
    wa(i+1) = (tau_vector(i)/(m*l*l)) + (-g*sin(theta(i+1))/l) -b*w(i+1); % Next acceleration
end

% Theta Plots
figure(1)
set(gcf, 'Position',  [700, 100, 500, 500])
subplot(2,1,1);
plot(time,rad2deg(theta)); % Plotting Theta graph
xlabel('Time')
ylabel('Theta')
title('Theta Result (deg)');
grid on
subplot(2,1,2);
plot(time,tau_vector); % Plotting Tau
title('Tau (Nm)');
xlabel('Time')
ylabel('Tau')
grid on
hold off
%exportgraphics(gcf, 'plot.pdf', 'ContentType', 'vector');

%% Animation
figure(2)
a = axes;
set(gcf, 'Position',  [100, 100, 520, 500])
grid on;
for i=1:length(w)-1
    P = [sin(theta(i)) -cos(theta(i))]*l;
    plot(a,[0,P(1)], [0,P(2)], 'Linewidth', 2, 'color', 'r'); % rod
    hold on
    plot(a,P(1),P(2),'.', 'MarkerSize',30); % ball
    text(a, -1, -1, ['t: ' num2str(i)]);
    text(a, 0, -1, ['theta: ' sprintf('%.3f', theta(i))]);
    text(a, 1, -1, ['w: ' sprintf('%.5f', w(i+1))]);
    text(a, 1, -0.5, ['tau: ' sprintf('%.5f', tau_vector(i+1))]);

    hold off
    
    title(a, 'Animation') 
    axis(a, 'equal')
    axis(a, [-1.5 1.5 -1.5 0.5]);
    grid on;
    pause(0.01);
end