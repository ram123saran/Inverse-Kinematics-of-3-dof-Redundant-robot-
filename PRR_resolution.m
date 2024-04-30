clear;
clc

% Link lengths 
L2 = 2;
L3 = 1.5;
theta1 = 0;
q = [0; 0; 0]; 
xd = [3; 1.5]; 

N = 1000;
T = 10;

alpha = 2; % deceleration factor
delta_t = T / N;

% Initialize arrays to store joint angles and time 
time = linspace(0, T, N);
q1_traj = zeros(1, N);
q2_traj = zeros(1, N);
q3_traj = zeros(1, N);
x_k_traj = zeros(2, N);

% Function to calculate forward kinematics 
forward_kinematics = @(L1, theta2, theta3) [L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3); 
                                                L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3)]; 

% Iterative inverse kinematics using Jacobian Transpose method
for i = 1:N
    % Calculate current end effector position
    x_k = forward_kinematics(q(1), q(2), q(3));
    x_k_traj(:, i) = x_k;
    L1 = q(1);
    % Calculate desired end effector velocity
    x_dot = alpha * (xd - x_k) / ((N - i + 1) * delta_t);
    q(1) = theta1;
    % Calculate Jacobian matrix
    J11 = -L1 * sin(q(1)) - L2 * sin(q(1) + q(2)) - L3 * sin(q(1) + q(2) + q(3));
    J12 = -L2 * sin(q(1) + q(2)) - L3 * sin(q(1) + q(2) + q(3));
    J13 = -L3 * sin(q(1) + q(2) + q(3));
    J21 = L1 * cos(q(1)) + L2 * cos(q(1) + q(2)) + L3 * cos(q(1) + q(2) + q(3));
    J22 = L2 * cos(q(1) + q(2)) + L3 * cos(q(1) + q(2) + q(3));
    J23 = L3 * cos(q(1) + q(2) + q(3));
    J = [J11, J12, J13;
         J21, J22, J23];
    
    % Calculate pseudo-inverse of Jacobian matrix
    J_pseudo_inv = pinv(J);
    
    q_dot = J_pseudo_inv * x_dot;
    
    q = q + q_dot * delta_t;

    % Store joint angles
    q1_traj(i) = q(1);
    q2_traj(i) = q(2);
    q3_traj(i) = q(3);
end

% Plot joint angles
figure;
subplot(2,1,1);
plot(time, q1_traj, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, q2_traj, 'g-', 'LineWidth', 1.5);
plot(time, q3_traj, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('q1', 'q2', 'q3');
title('Joint Angles Trajectory');
grid on;
subplot(2,1,2);
plot(x_k_traj(1, :), x_k_traj(2, :), 'b-', 'LineWidth', 1.5);
hold on;
plot(xd(1), xd(2), 'ro', 'MarkerSize', 7,'MarkerFaceColor', 'r');
xlabel('X');
ylabel('Y');
title('End Effector Trajectory');
grid on;
% joint angles
display(q) 
% end position 
display(x_k) 
