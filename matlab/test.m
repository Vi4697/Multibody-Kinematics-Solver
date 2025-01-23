% test.m
% This script tests if the constraints defined in constraints.m are correct.

% Load necessary data
run('data.m'); % Load mechanism configuration data

% Initial setup
q = zeros(24, 1); % Initial guess for the state variables
t = 0; % Initial time

% Initial configuration (example values, ensure alignment with the mechanism)
q = [
    2.2; -0.4; 0;       % r1 (D) and fi1
    2.5; -1.4; 0;       % r2 (C) and fi2
    2.9; -1.9; 0;       % r3 (A) and fi3
    2.9; -1.1; 0;       % r4 (B) and fi4
    0.1; -0.8; 0;       % r5 (N) and fi5
    1.9; -1.4; 0;       % r6 (M) and fi6
    0.4; -0.2; 0;       % r7 (H) and fi7
    1.6;  0.4; 0;       % r8 (G) and fi8
];

% Evaluate constraints
F = constraints(q, t);

% Display constraint results
disp('Constraint Values:');
disp(F);

% Check if all constraints are satisfied
tolerance = 1e-6; % Numerical tolerance for validation
if all(abs(F) < tolerance)
    disp('All constraints are satisfied at the initial configuration.');
else
    disp('Some constraints are not satisfied. Debugging the discrepancies:');
    for i = 1:length(F)
        if abs(F(i)) >= tolerance
            fprintf('Constraint %d is not satisfied.\n', i);
            fprintf('F(%d): %f\n', i, F(i));
        end
    end
end
