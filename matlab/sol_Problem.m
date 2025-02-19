function [T, Q, DQ, D2Q] = sol_Problem()
% Solves the kinematics problem (position, velocity, acceleration) for a four-bar mechanism.
%
% Outputs:
%   T   - Array of time instances.
%   Q   - Matrix containing positions at each time instance.
%   DQ  - Matrix containing velocities at each time instance.
%   D2Q - Matrix containing accelerations at each time instance.



%%  Select the desired point for plotting
% Each point in the mechanism has an index:
% 1 = D, 2 = C, 3 = A, 4 = B, 5 = N(ground), 6 = M, 7 = H(ground), 8 = G
P = 3;
%%



% Initialize the mechanism configuration at t = 0
% Define the absolute coordinates of all points in the mechanism.
q = [2.2; -0.4;  0;   % Coordinates of point D
     2.5; -1.4;  0;   % Coordinates of point C
     2.9; -1.9;  0;   % Coordinates of point A
     2.9; -1.1;  0;   % Coordinates of point B
     0.1; -0.8;  0;   % Coordinates of point N
     1.9; -1.4;  0;   % Coordinates of point M
     0.4; -0.2;  0;   % Coordinates of point H
     1.6;  0.4;  0];  % Coordinates of point G

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
    q = NewtonRaphson(q0, t, P);

    % Solve for velocities based on the updated positions
    dq = Velocity(q, t,P);

    % Solve for accelerations based on the updated positions and velocities
    d2q = Acceleration(dq, q, t, P);

    % Save the results for the current time step
    counter = counter + 1;
    T(1, counter) = t;       % Store the current time
    Q(:, counter) = q;       % Store positions
    DQ(:, counter) = dq;     % Store velocities
    D2Q(:, counter) = d2q;   % Store accelerations
end



% Plot the position results for the selected point
figure(1)
subplot(2, 1, 1)
plot(T, Q(((P - 1) * 3) + 1, :)); % X-coordinate of the selected point
grid on;
title('Position in X');
ylabel('Length [meters]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, Q(((P - 1) * 3) + 2, :)); % Y-coordinate of the selected point
grid on;
title('Position in Y');
ylabel('Length [meters]');
xlabel('Time [sec]');

% Plot the velocity results for the selected point
figure(2)
subplot(2, 1, 1)
plot(T, DQ(((P - 1) * 3) + 1, :)); % X-velocity of the selected point
grid on;
title('Velocity in X');
ylabel('Length [meters/sec]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, DQ(((P - 1) * 3) + 2, :)); % Y-velocity of the selected point
grid on;
title('Velocity in Y');
ylabel('Length [meters/sec]');
xlabel('Time [sec]');

% Plot the acceleration results for the selected point
figure(3)
subplot(2, 1, 1)
plot(T, D2Q(((P - 1) * 3) + 1, :)); % X-acceleration of the selected point
grid on;
title('Acceleration in X');
ylabel('Length [meters/sec^2]');
xlabel('Time [sec]');

subplot(2, 1, 2)
plot(T, D2Q(((P - 1) * 3) + 2, :)); % Y-acceleration of the selected point
grid on;
title('Acceleration in Y');
ylabel('Length [meters/sec^2]');
xlabel('Time [sec]');
end
