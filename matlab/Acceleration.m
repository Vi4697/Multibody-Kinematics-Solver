function d2q = Acceleration(dq, q, t, P)
% d2q = Acceleration(dq, q, t)
% This function solves the acceleration problem for a multibody mechanism.
% It calculates the second time derivatives (accelerations) of the absolute coordinates.
%
% Inputs:
%   dq - Vector of first time derivatives (velocities) of absolute coordinates
%   q  - Vector of absolute coordinates (positions and angles)
%   t  - Current time instant
% Outputs:
%   d2q - Vector of second time derivatives (accelerations) of absolute coordinates

    % Step 1 Load mechanism dimensions and auxiliary quantities from data file
    data; % This file contains information about joint positions, vectors, and constraints.

    % Step 2 Define a constant skew-symmetric matrix used for rotation-related calculations
    Om = [0 -1; 1 0]; % This matrix helps compute perpendicular vectors in 2D space.

    % Step 3 Extract positions and angles from the input vector q
    % Positions (r1 to r8) and angles (fi1 to fi8) for different joints or bodies
    r1 = q(1:2); fi1 = q(3);    % Frame 1: Position (X1, Y1) and rotation angle (Fi1)
    r2 = q(4:5); fi2 = q(6);    % Frame 2: Position (X2, Y2) and rotation angle (Fi2)
    r3 = q(7:8); fi3 = q(9);    % Frame 3: Position (X3, Y3) and rotation angle (Fi3)
    r4 = q(10:11); fi4 = q(12); % Frame 4: Position (X4, Y4) and rotation angle (Fi4)
    r5 = q(13:14); fi5 = q(15); % Frame 5: Position (X5, Y5) and rotation angle (Fi5)
    r6 = q(16:17); fi6 = q(18); % Frame 6: Position (X6, Y6) and rotation angle (Fi6)
    r7 = q(19:20); fi7 = q(21); % Frame 7: Position (X7, Y7) and rotation angle (Fi7)
    r8 = q(22:23); fi8 = q(24); % Frame 8: Position (X8, Y8) and rotation angle (Fi8)

    % Step 4 Compute rotation matrices for each joint (used to map local coordinates to global coordinates)
    Rot1 = Rot(fi1); Rot2 = Rot(fi2); Rot3 = Rot(fi3);
    Rot4 = Rot(fi4); Rot5 = Rot(fi5); Rot6 = Rot(fi6);
    Rot7 = Rot(fi7); Rot8 = Rot(fi8);

    % Step 5 Extract velocities (dq) from the input vector dq
    % Velocities (linear: dr1 to dr8, angular: dfi1 to dfi8)
    dr1 = dq(1:2); dfi1 = dq(3);    % Velocity of Frame 1
    dr2 = dq(4:5); dfi2 = dq(6);    % Velocity of Frame 2
    dr3 = dq(7:8); dfi3 = dq(9);    % Velocity of Frame 3
    dr4 = dq(10:11); dfi4 = dq(12); % Velocity of Frame 4
    dr5 = dq(13:14); dfi5 = dq(15); % Velocity of Frame 5
    dr6 = dq(16:17); dfi6 = dq(18); % Velocity of Frame 6
    dr7 = dq(19:20); dfi7 = dq(21); % Velocity of Frame 7
    dr8 = dq(22:23); dfi8 = dq(24); % Velocity of Frame 8

    % Step 6 Initialize the right-hand side vector `gam` for acceleration equations
    gam = zeros(24, 1); % 24 constraints in total for the mechanism

    % Step 7 --- Revolute joint contributions to acceleration equations ---
    % Revolute joints eliminate two degrees of freedom, introducing two constraints per joint.
    gam(1:2) = -Rot1 * sB01 * dfi1^2; % Joint O-D (Frames 0, 1): Accounts for rotational acceleration
    gam(3:4) = -Rot7 * sB07 * dfi7^2; % Joint O-H (Frames 0, 7)
    gam(5:6) = -Rot5 * sB05 * dfi5^2; % Joint O-N (Frames 0, 5)
    gam(7:8) = Rot8 * sA82 * dfi8^2 - Rot2 * sB82 * dfi2^2; % Joint G-C (Frames 8, 2)
    gam(9:10) = Rot6 * sA61 * dfi6^2 - Rot1 * sB61 * dfi1^2; % Joint M-D (Frames 6, 1)
    gam(11:12) = Rot1 * sA12 * dfi1^2 - Rot2 * sB12 * dfi2^2; % Joint D-C (Frames 1, 2)
    gam(13:14) = Rot2 * sA24 * dfi2^2 - Rot4 * sB24 * dfi4^2; % Joint C-B (Frames 2, 4)
    gam(15:16) = Rot4 * sA43 * dfi4^2 - Rot3 * sB43 * dfi3^2; % Joint B-A (Frames 4, 3)
    gam(17:18) = Rot1 * sA13 * dfi1^2 - Rot3 * sB13 * dfi3^2; % Joint D-A (Frames 1, 3)

    % Step 8 --- Translational joint contributions to acceleration equations ---
    % Translational joints eliminate two degrees of freedom (one for translation, one for orientation)

    gam(19) = 0; % Angular acceleration is zero because translational joints do not allow relative rotation
    % Computes linear acceleration for the translational joint 6-5 using relative velocity and rotation effects
    gam(20) = (Rot6 * v56)' * (2 * Om * (dr6 - dr5) * dfi6 + (r6 - r5) * dfi6^2 - Rot5 * sA56 * (dfi6 - dfi5)^2);
    
    
    gam(21) = 0; 
    gam(22) = (Rot8 * v78)' * (2 * Om * (dr8 - dr7) * dfi8 + (r8 - r7) * dfi8^2 - Rot7 * sA78 * (dfi8 - dfi7)^2);

    % Step 9 --- Driving constraint contributions ---
    % Driving constraints specify the desired motion of pistons or cylinders over time
    % These constraints depend on time derivatives of predefined motion functions

    if ismember(P, [1, 2, 3, 4, 5,6])
        Dr_con1 = -0.225 * sin(1.5 * t);
        Dr_con2 = -0.1125 * sin(1.5 * t);
    elseif ismember(P, [7])
        Dr_con1 = 0.225 * sin(1.5 * t);
        Dr_con2 =  0.1125 * sin(1.5 * t);
    elseif ismember(P, [ 8])
        Dr_con1 =- 0.225 * sin(1.5 * t);
        Dr_con2 =  0.1125 * sin(1.5 * t);
    else
        error('Invalid P');
    end


    gam(23) = (Rot6 * u56)' * (2 * Om * (dr6 - dr5) * dfi6 + (r6 - r5) * dfi6^2 - Rot5 * sA56 * (dfi6 - dfi5)^2) ...
              - (Dr_con1); % Driving constraint for piston/cylinder (6-5)
    % Computes the acceleration for piston/cylinder pair (6-5).
    % The left term calculates dynamic contributions (velocity and rotation effects).
    % The right term specifies the predefined sinusoidal driving acceleration.


    gam(24) = (Rot8 * u78)' * (2 * Om * (dr8 - dr7) * dfi8 + (r8 - r7) * dfi8^2 - Rot7 * sA78 * (dfi8 - dfi7)^2) ...
              - (Dr_con2); % Driving constraint for piston/cylinder (8-7)

    % Step 10 --- Calculate the Jacobian matrix ---
    % The Jacobian matrix defines how the constraint equations change with respect to positions (q).
    % It is used to relate changes in positions to accelerations.
    Fq = Jacobian(q); % Compute the Jacobian matrix for the current configuration.

    % Step 11 --- Solve the linear system ---
    % Solve the equation Fq * d2q = gam to find accelerations (d2q).
    % d2q contains the second derivatives (accelerations) of positions and angles.
    d2q = Fq \ gam; % Compute accelerations using the Jacobian matrix and gam vector.
    % gives the linear and angular accelerations for all components. Ensures that the mechanism respects all constraints during motion.
end
