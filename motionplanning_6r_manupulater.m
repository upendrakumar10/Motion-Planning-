%%%% Motion Planning for 6R Manupulater planning path from starting to
%%%% destination point using Inverse kinematics
% Define the start and end points of the robot motion
clear all
clc
start_point = [0, 0];
end_point = [5, 5];
% Plan a path (simple straight line)
path = [linspace(start_point(1), end_point(1), 100); linspace(start_point(2), end_point(2), 100)]';
% Define 6R manipulator parameters (link lengths and joint angles)
L1 = 2; L2 = 1.5; L3 = 1; L4 = 1; L5 = 0.5; L6 = 0.5; % link lengths
theta1 = linspace(0, pi, 100); % joint 1 angles
% Initialize figure
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
title('Robot Motion Planning with 6R Manipulator');
% Animation loop
for i = 1:size(path, 1)
    % Update robot position
    plot(path(i, 1), path(i, 2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Update manipulator configuration
    x = path(i, 1);
    y = path(i, 2);
    % Compute inverse kinematics for joint 2 and 3 angles
    r = sqrt(x^2 + y^2);
    beta = atan2(y, x);
    D = (r^2 + L2^2 - L3^2) / (2 * L2 * r);
    % Ensure D is within [-1, 1] to avoid complex results
    D = max(min(D, 1), -1);
    theta3 = acos(D);
    % Check if theta3 is real, if not, set it to 0
    if ~isreal(theta3)
        theta3 = 0;
    end
    theta2 = beta - atan2(L3*sin(theta3), L2 + L3*cos(theta3));
    
    % Plot manipulator links
    x0 = 0; y0 = 0; % base position
    x1 = L1 * cos(theta1(i)); y1 = L1 * sin(theta1(i));
    x2 = x1 + L2 * cos(theta2); y2 = y1 + L2 * sin(theta2);
    x3 = x2 + L3 * cos(theta2 + theta3); y3 = y2 + L3 * sin(theta2 + theta3);
    x4 = x3 + L4 * cos(theta2 + theta3); y4 = y3 + L4 * sin(theta2 + theta3);
    x5 = x4 + L5 * cos(theta2 + theta3); y5 = y4 + L5 * sin(theta2 + theta3);
    x6 = x5 + L6 * cos(theta2 + theta3); y6 = y5 + L6 * sin(theta2 + theta3);
    
    % Plot manipulator joints and links
    plot([x0, x1, x2, x3, x4, x5, x6], [y0, y1, y2, y3, y4, y5, y6], 'k-', 'LineWidth', 2);
    plot([x0, x1], [y0, y1], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
    plot([x2, x3], [y2, y3], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
    plot([x4, x5], [y4, y5], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
    plot([x6], [y6], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
%     plot([x0, x1, x2, x3, x4, x5, x6], [y0, y1, y2, y3, y4, y5, y6], 'k-', 'LineWidth', 2);
%     plot([x0, x1], [y0, y1], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
%     plot([x2, x3], [y2, y3], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
%     plot([x4, x5], [y4, y5], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
%     plot([x6], [y6], 'ko', 'MarkerSize', 5, 'LineWidth', 2);
    
    % Adjust axes limits if necessary
    xlim([-1, 6]);
    ylim([-1, 6]);
    
    drawnow;
    pause(0.05); % Adjust animation speed
end
% Plot the final robot position
plot(end_point(1), end_point(2), 'go', 'MarkerSize', 1, 'LineWidth', 2);
legend('Start', 'End', 'Location', 'Best');