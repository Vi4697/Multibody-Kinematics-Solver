function [T, Q, DQ, D2Q] = sol_Problem_6()
% Solves the kinematics problem (position, velocity, acceleration) for a four-bar mechanism.
%
% Outputs:
%   T   - Array of time instances.
%   Q   - Matrix containing positions at each time instance.
%   DQ  - Matrix containing velocities at each time instance.
%   D2Q - Matrix containing accelerations at each time instance.

% Initialize the mechanism configuration at t = 0
% Define the absolute coordinates of all points in the mechanism.
q = [
    2.2; -0.4; 0;        % r1 (D) and fi1: Global position and orientation of Body 1.
    2.5; -1.4; 0;        % r2 (C) and fi2: Global position and orientation of Body 2.
    2.9; -1.9; 0;        % r3 (A) and fi3: Global position and orientation of Body 3.
    2.9; -1.1; 0;        % r4 (B) and fi4: Global position and orientation of Body 4.
    0.1; -0.8; 0;        % r5 (N) and fi5: Global position and orientation of Body 5.
    1.9; -1.4; 0;        % r6 (M) and fi6: Global position and orientation of Body 6.
    0.4; -0.2; 0;        % r7 (H) and fi7: Global position and orientation of Body 7.
    1.6;  0.4; 0;        % r8 (G) and fi8: Global position and orientation of Body 8.
];


% Initialize velocity and acceleration vectors
% Assume the mechanism is initially at rest (dq = 0, d2q = 0).
dq = zeros(24, 1);     % Velocity vector
d2q = zeros(24, 1);    % Acceleration vector

% Time step and simulation parameters
dt = 0.01;             % Time step for simulation (seconds)
counter = 0;           % Counter for storing results

% Loop through each time step to solve kinematics
for t = 0:dt:5
    % Estimate the initial position approximation for Newton-Raphson
    % Use previous position, velocity, and acceleration to predict the next position.
    q0 = q + dq * dt + 0.5 * d2q * dt^2;

    % Solve for the corrected position using the Newton-Raphson method
    q = NewtonRaphson(q0, t); 

    % Solve for velocities based on the updated positions
    dq = Velocity(q, t);

    % Solve for accelerations based on the updated positions and velocities
    d2q = Acceleration(dq, q, t);

    % Save the results for the current time step
    counter = counter + 1;
    T(1, counter) = t;       % Store the current time
    Q(:, counter) = q;       % Store positions
    DQ(:, counter) = dq;     % Store velocities
    D2Q(:, counter) = d2q;   % Store accelerations
end



%%  Select the desired point for plotting
% Each point in the mechanism has an index:
% 1 = D, 2 = C, 3 = A, 4 = B, 5 = N, 6 = M, 7 = H, 8 = G
P = 8;  % Example: P = 4 corresponds to point B.
%%



%% Validate the indexing
% Debug: Ensure dimensions and indexing are consistent
disp('Number of points in Q: ' + string(size(Q, 1) / 3));
disp('Number of time steps: ' + string(size(Q, 2)));

%% Select the desired point for plotting
% Each point in the mechanism has an index:
% 1 = D, 2 = C, 3 = A, 4 = B, 5 = N, 6 = M, 7 = H, 8 = G
P = 8;  % Example: P = 4 corresponds to point B.

% Calculate indices for the selected point
x_index = ((P - 1) * 3) + 1; % X-coordinate index
y_index = ((P - 1) * 3) + 2; % Y-coordinate index

% Debug: Check the selected indices
disp('Selected point: ' + string(P));
disp('X-index: ' + string(x_index));
disp('Y-index: ' + string(y_index));

%% Plot the position results for the selected point
figure(1)
subplot(2, 1, 1)
plot(T, Q(x_index, :)); % X-coordinate of the selected point
grid on;
title('Position in X');
ylabel('Length [meters]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, Q(y_index, :)); % Y-coordinate of the selected point
grid on;
title('Position in Y');
ylabel('Length [meters]');
xlabel('Time [sec]');

%% Plot the velocity results for the selected point
figure(2)
subplot(2, 1, 1)
plot(T, DQ(x_index, :)); % X-velocity of the selected point
grid on;
title('Velocity in X');
ylabel('Length [meters/sec]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, DQ(y_index, :)); % Y-velocity of the selected point
grid on;
title('Velocity in Y');
ylabel('Length [meters/sec]');
xlabel('Time [sec]');

%% Plot the acceleration results for the selected point
figure(3)
subplot(2, 1, 1)
plot(T, D2Q(x_index, :)); % X-acceleration of the selected point
grid on;
title('Acceleration in X');
ylabel('Length [meters/sec^2]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, D2Q(y_index, :)); % Y-acceleration of the selected point
grid on;
title('Acceleration in Y');
ylabel('Length [meters/sec^2]');
xlabel('Time [sec]');
end