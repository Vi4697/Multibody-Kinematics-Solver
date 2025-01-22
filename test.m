% test.m
% This script tests if the constraints defined in constraints.m are correct.


% Load necessary data
run('data.m');          % Load mechanism's configuration data

% Initial setup
q = zeros(24, 1);       % Initial guess for the state variables (adjust size to match your DOF)
t = 0;                  % Initial time (adjust if time-dependent constraints exist)
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
% Extract positions and angles from q
r1 = q(1:2); fi1 = q(3);   % Component 1 (e.g., point D)
r2 = q(4:5); fi2 = q(6);   % Component 2 (e.g., point C)
r3 = q(7:8); fi3 = q(9);   % Component 3 (e.g., point A)
r4 = q(10:11); fi4 = q(12); % Component 4 (e.g., point B)
r5 = q(13:14); fi5 = q(15); % Component 5 (e.g., piston 5)
r6 = q(16:17); fi6 = q(18); % Component 6 (e.g., cylinder 6)
r7 = q(19:20); fi7 = q(21); % Component 7 (e.g., piston 7)
r8 = q(22:23); fi8 = q(24); % Component 8 (e.g., cylinder 8)

% Compute rotation matrices for each component
Rot1 = Rot(fi1); Rot2 = Rot(fi2); Rot3 = Rot(fi3);
Rot4 = Rot(fi4); Rot5 = Rot(fi5); Rot6 = Rot(fi6);
Rot7 = Rot(fi7); Rot8 = Rot(fi8);

% Evaluate constraints
F = constraints(q, t); % F: Constraint vector

% Display constraint results
disp('Constraint Values:');
disp(F);

% Check if all constraints are satisfied
tolerance = 1e-6; % Numerical tolerance for checking
if all(abs(F) < tolerance)
    disp('All constraints are satisfied at the initial configuration.');
else
    disp('Some constraints are not satisfied. Check the following discrepancies:');
    for i = 1:2:length(F)
        if abs(F(i)) >= tolerance || abs(F(i+1)) >= tolerance
            fprintf('Constraint %d is not satisfied.\n', ceil(i/2));
            fprintf('F(%d): %f, F(%d): %f\n', i, F(i), i+1, F(i+1));
            disp('Debugging Values:');
            if i == 1
                % Debug F(1:2) (0-1 joint)
                disp('Constraint 0-1 Debugging (F(1:2)):');
                disp('Expected:');
                disp(sA01);
                disp('Calculated:');
                disp(r1 + Rot1 * sB01);
                disp('Difference:');
                disp(sA01 - (r1 + Rot1 * sB01));
            elseif i == 3
                % Debug F(3:4) (1-3 joint)
                disp('Constraint 1-3 Debugging (F(3:4)):');
                disp('Expected:');
                disp(sA13);
                disp('Calculated:');
                disp(r1 + Rot1 * sA13 - (r3 + Rot3 * sB13));
                disp('Difference:');
                disp(sA13 - (r3 + Rot3 * sB13));
            elseif i == 5
                % Debug F(5:6) (3-4 joint)
                disp('Constraint 3-4 Debugging (F(5:6)):');
                disp('Expected:');
                disp(sA34);
                disp('Calculated:');
                disp(r3 + Rot3 * sA34 - (r4 + Rot4 * sB34));
                disp('Difference:');
                disp(sA34 - (r4 + Rot4 * sB34));
            elseif i == 7
                % Debug F(7:8) (4-2 joint)
                disp('Constraint 4-2 Debugging (F(7:8)):');
                disp('Expected:');
                disp(sA42);
                disp('r4:');
                disp(r4);
                disp('r2:');
                disp(r2);
                disp('Rot4:');
                disp(Rot4);
                disp('Rot2:');
                disp(Rot2);
                disp('Calculated:');
                disp(r4 + Rot4 * sA42 - (r2 + Rot2 * sB42));
                disp('Difference:');
                disp(sA42 - (r2 + Rot2 * sB42));
            elseif i == 9
                % Debug F(9:10) (2-8 joint)
                disp('Constraint 2-8 Debugging (F(9:10)):');
                disp('Expected:');
                disp(sA28);
                disp('Calculated:');
                disp(r2 + Rot2 * sA28 - (r8 + Rot8 * sB28));
                disp('Difference:');
                disp(sA28 - (r8 + Rot8 * sB28));
            end
        end
    end
end
