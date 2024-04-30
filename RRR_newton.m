clear
clc 

% Link lengths 
L1 = 2;
L2 = 1.5;
L3 = 1;

% Desired end effector position 
x_target = 3; 
y_target = 3;  

% Initial joint angles 
theta1 = 0;  
theta2 = 0;  
theta3 = 0;  

x_k_traj = zeros(2);
% Tolerance for convergence 
tolerance = 1e-5; 


error_x = 10;
error_y = 10;
% Function to calculate forward kinematics 
forward_kinematics = @(theta1, theta2, theta3) [L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
                                                L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3)];
i = 1; 
% Iterative inverse kinematics using Jacobian Transpose method
while(abs(error_x) > tolerance && abs(error_y) > tolerance)
    % Calculate current end effector position
    current_pos = forward_kinematics(theta1, theta2, theta3);
    x_current = current_pos(1);
    y_current = current_pos(2);
    x_k_traj(:, i) = [x_current;y_current];
    i = i+1; 
    % Error between desired and current end effector position
    error_x = x_target - x_current;
    error_y = y_target - y_current;
    
    % Calculate Jacobian matrix
    J11 = -L1 * sin(theta1) - L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3);
    J12 = -L2 * sin(theta1 + theta2) - L3 * sin(theta1 + theta2 + theta3);
    J13 = -L3 * sin(theta1 + theta2 + theta3);
    J21 = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
    J22 = L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3);
    J23 = L3 * cos(theta1 + theta2 + theta3);
    J = [J11, J12, J13;
         J21, J22, J23];
    
    % Calculate pseudo-inverse of Jacobian matrix
    J_pseudo_inv = pinv(J);
    
    % Calculate delta_theta
    delta_theta = J_pseudo_inv * [error_x; error_y];
    
    % Update joint angles
    theta1 = theta1 + delta_theta(1);
    theta2 = theta2 + delta_theta(2);
    theta3 = theta3 + delta_theta(3);
end
figure;
plot(x_k_traj(1, :), x_k_traj(2, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(x_target, y_target, 'ro', 'MarkerSize', 7,'MarkerFaceColor', 'r');
xlabel('X');
ylabel('Y');
title('End Effector Trajectory');
grid on;
% Print the final joint angles 
fprintf('Final Joint Angles:\n');
fprintf('Theta1: %f\n', theta1);
fprintf('Theta2: %f\n', theta2);
fprintf('Theta3: %f\n', theta3);

% checking the result 
x = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3)
y = L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3)