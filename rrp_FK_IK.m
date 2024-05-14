%% RRP - 3 DOF Robotic Leg Kinematics
clear
clc
close all
syms q1 q2 q3 Rx s_joints

% Original config
%s_joints = 0; % Add space between the joints, link lenght.
%Rx = eye(4); % Rotate the matrix, if = eye(4), is not rotating.

% For a better visualization, rotate the base frame Rx(-90º)
% and add a space of 2 between the joints (s_joints).
s_joints = 2; % Add space of 2 between the joints.
Rx = [1 0 0 0;0 0 -1 0;0 1 0 0;0 0 0 1]; % Rotate by Rx(90º)

A1 = Rx*[1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1];

A12 = Rx*[ cos(q1) 0 -sin(q1) 0
        sin(q1) 0 cos(q1) s_joints
        0 -1 0 0
        0 0 0 1];

A23 = [ cos(q2) 0 -sin(q2) s_joints
        sin(q2) 0 cos(q2) 0
        0 -1 0 0
        0 0 0 1];

A34 = [ 1 0 0 0
        0 1 0 0
        0 0 1 q3
        0 0 0 1];

A13 = A12*A23;
A14 = A13*A34

%% Plotting
% Parameters to appear like the picture
q1 = (pi/2);
q2 = (-pi/2);
q3 = 2;

% Calling the plot function
A = {A1, A12, A13, A14};
q = [q1 q2 q3];
figure(1)
plotManipulator(A,q)
%exportgraphics(gcf, 'plot.pdf', 'ContentType', 'vector');

%% Inverse kinematics
% Generating random values
yc = rand()*6 - 3; % Random y value between -3 and 3
xc = rand()*6 - 3; % Random x value between -3 and 3
zc = rand()*8 + 2; % Random z value between 2 and 8

% IK with s_joints = 0 and Rx = I
%q1 = atan2(yc,xc);
%q2 = atan2(yc,zc*sin(q1));
%q3 = -zc/(cos(q2));

% IK with Rx(90º) and s_joints != 0
q1 = atan2(zc-s_joints,xc);
q2 = -atan2(xc-s_joints*cos(q1),yc*cos(q1));
q3 = yc/(cos(q2));

q = [q1 q2 q3]
pose = [xc yc zc]
A = {A1, A12, A13, A14};
figure(2)
plotManipulator(A,q);

%% Plot workspace
step = 0.5; % Increase step for more points
q1stream = -pi:step:pi;
q2stream = -pi:step:pi;
q3stream = 1:step:2;
n_dots = length(q1stream)*length(q2stream)*length(q3stream); % Number of dots
X = ['Number of points: ',num2str(n_dots), '. Plotting workspace... '];
disp(X)

dots_workspace = zeros(n_dots,3); % Creating the workspace matrix
cnt = 1; % Counter
aux = A14(1:3,4); % Auxiliar variable

% Compute x,y,z given several q1, q2, q3. This may take a while.
for i = 1:length(q1stream)
    for j = 1:length(q2stream)
        for k = 1:length(q3stream)
            q1 = q1stream(i);
            q2 = q2stream(j);
            q3 = q3stream(k);
            vec = subs(aux);
            dots_workspace(cnt,:) = vec;
            cnt = cnt + 1;
        end
    end
end

% Extracting coordinates
x = dots_workspace(:, 1); 
y = dots_workspace(:, 2); 
z = dots_workspace(:, 3);

% Plotting the workspace
figure(3)
daspect([1 1 1]); % Aspect ratio
plot3(x, y, z, '.','MarkerSize',1); % Dots that compose the workspace.
hold on
scatter3(0,0,0, 'filled','MarkerFaceColor','r'); % Center, robot base is there.
hold off
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace');
grid on;
%exportgraphics(gcf, 'plot.pdf', 'ContentType', 'vector');

%% plot Manipulator function
function plotManipulator(A,q)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    A1 = A{1};
    A12 = A{2};
    A13 = A{3};
    A14 = A{4};

    % Extracting values
    A1 = subs(A1);
    A1pos = A1(:,end);
    A1rot = A1(1:3,1:3);
    
    A12 = subs(A12);
    A12pos = A12(:,end);
    A12rot = A12(1:3,1:3);
    
    A13 = subs(A13);
    A13pos = A13(:,end);
    A13rot = A13(1:3,1:3);
    
    A14 = subs(A14);
    A14pos = A14(:,end);
    A14rot = A14(1:3,1:3);
    
    % Plotting    
    hold on
    % Set axis limits
    xlim([-5 5]); % Set x-axis limits
    ylim([-5 5]); % Set y-axis limits
    zlim([0 10]); % Set z-axis limits
    daspect([1 1 1]); % Aspect ratio
    view(3)
    
    % Frame coordinates -Base - Joint 1
    quiver3(A1pos(1), A1pos(2), A1pos(3), A1rot(1, 1), A1rot(2, 1), A1rot(3, 1), 'r', 'LineWidth', 2, 'MaxHeadSize', 1); % X
    quiver3(A1pos(1), A1pos(2), A1pos(3), A1rot(1, 2), A1rot(2, 2), A1rot(3, 2), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Y
    quiver3(A1pos(1), A1pos(2), A1pos(3), A1rot(1, 3), A1rot(2, 3), A1rot(3, 3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Z
    
    % Frame coordinates - Joint 2
    quiver3(A12pos(1), A12pos(2), A12pos(3), A12rot(1, 1), A12rot(2, 1), A12rot(3, 1), 'r', 'LineWidth', 2, 'MaxHeadSize', 1); % X
    quiver3(A12pos(1), A12pos(2), A12pos(3), A12rot(1, 2), A12rot(2, 2), A12rot(3, 2), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Y
    quiver3(A12pos(1), A12pos(2), A12pos(3), A12rot(1, 3), A12rot(2, 3), A12rot(3, 3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Z
    
    % Frame coordinates - Joint 3
    quiver3(A13pos(1), A13pos(2), A13pos(3), A13rot(1, 1), A13rot(2, 1), A13rot(3, 1), 'r', 'LineWidth', 2, 'MaxHeadSize', 1); % X
    quiver3(A13pos(1), A13pos(2), A13pos(3), A13rot(1, 2), A13rot(2, 2), A13rot(3, 2), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Y
    quiver3(A13pos(1), A13pos(2), A13pos(3), A13rot(1, 3), A13rot(2, 3), A13rot(3, 3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Z
    
    % Frame coordinates - End-effector
    quiver3(A14pos(1), A14pos(2), A14pos(3), A14rot(1, 1), A14rot(2, 1), A14rot(3, 1), 'r', 'LineWidth', 2, 'MaxHeadSize', 1); % X
    quiver3(A14pos(1), A14pos(2), A14pos(3), A14rot(1, 2), A14rot(2, 2), A14rot(3, 2), 'g', 'LineWidth', 2, 'MaxHeadSize', 1); % Y
    quiver3(A14pos(1), A14pos(2), A14pos(3), A14rot(1, 3), A14rot(2, 3), A14rot(3, 3), 'b', 'LineWidth', 2, 'MaxHeadSize', 1); % Z
    
    % Lines ("links")
    x12 = [A1pos(1,end) A12pos(1,end)];
    y12 = [A1pos(2,end) A12pos(2,end)];
    z12 = [A1pos(3,end) A12pos(3,end)];
    
    x23 = [A13pos(1,end) A12pos(1,end)];
    y23 = [A13pos(2,end) A12pos(2,end)];
    z23 = [A13pos(3,end) A12pos(3,end)];
    
    x34 = [A13pos(1,end) A14pos(1,end)];
    y34 = [A13pos(2,end) A14pos(2,end)];
    z34 = [A13pos(3,end) A14pos(3,end)];
    
    plot3(x23,y23,z23, 'LineWidth', 1,'color','k');
    plot3(x12,y12,z12, 'LineWidth',1,'color','k');
    plot3(x34,y34,z34, 'LineWidth',1,'color','k');
    
    % Frames centers
    scatter3(A1pos(1,end),A1pos(2,end),A1pos(3,end), 'filled','MarkerFaceColor','r');
    scatter3(A12pos(1,end),A12pos(2,end),A12pos(3,end), 'filled','MarkerFaceColor','b');
    scatter3(A13pos(1,end),A13pos(2,end),A13pos(3,end), 'filled','MarkerFaceColor','g');
    scatter3(A14pos(1,end),A14pos(2,end),A14pos(3,end), 'filled','MarkerFaceColor','y');
    
    % Label and View angle
    xlabel('X')
    ylabel('Z')
    zlabel('Y')
    grid on;
    hold off
end
