%% Slider crank linkage (1-RRPR) - Closed Chain Kinematics

clear
clc
close all
syms d l1 l2 q1 q2 q3

% Physical Parameters
l1 = 3;
l2 = 4;

% Limits
maxq3 = l1 + l2; % Maximum q3 possible
minq3 = abs(l1-l2); % Minimum q3 possible
h_max = sqrt(l2*l2 - l1*l1/4); % Maximum height possible

% Generating random q3 (d)
%q3_objective = minq3 + (maxq3 - minq3) * rand(1)

% Generating random q1
q1_objective = rand()*pi

% Inverse kinematics
q3 = sqrt(l1*l1 + l2*l2 - 2*l1*l2*cos(q1_objective))

% Forward kinematics
q1 = acos((l1*l1 + l2*l2 - q3*q3)/(2*l1*l2))

% q3 tip position
q3pos = [cos(q1)*l2 sin(q1)*l2];

%% Plotting
figure
a = axes;
grid on;
axis(a, 'equal')
axis(a, [-maxq3-1 maxq3+1 -1 round(h_max+1)]);
q1pos = [0,0];
q2pos = [l1,0];

verify_l1 = sqrt((q2pos(1) - q1pos(1))^2 + (q2pos(2) - q1pos(2))^2)
verify_q3 = sqrt((q2pos(1) - q3pos(1))^2 + (q2pos(2) - q3pos(2))^2)

hold on

% Joints
plot(q1pos(1), q1pos(2),'.','MarkerSize',20,'Color','y')
plot(q2pos(1), q2pos(2),'.','MarkerSize',20,'Color','b')
plot(q3pos(1), q3pos(2),'.','MarkerSize',20,'Color','r')

% verify links distance would be good.

% Lines ("links")
x12 = [q1pos(1) q2pos(1)];
y12 = [q1pos(2) q2pos(2)];

x13 = [q1pos(1) q3pos(1)];
y13 = [q1pos(2) q3pos(2)];

x23 = [q2pos(1) q3pos(1)];
y23 = [q2pos(2) q3pos(2)];

plot(x12,y12,'LineWidth', 1,'color','k');
plot(x13,y13,'LineWidth',1,'color','k');
plot(x23,y23, 'LineWidth',1,'color','k');

xlabel('X');
ylabel('Y')
exportgraphics(gcf, 'plot.pdf', 'ContentType', 'vector');
