function Fq = Jacobian(q)
% Fq = Jacobian(q)
% This function calculates the Jacobian matrix for the constraint equations.
% It is used in the Newton-Raphson process to solve the position problem.
%
% Input:
%   q - The vector of absolute coordinates, including positions (x, y) and angles (fi).
% Output:
%   Fq - The constraint Jacobian matrix.

    % Step 1: Load mechanism data
    % This includes the mechanism's dimensions and auxiliary quantities such as joint offsets.
    data; 

    % Step 2: Define a constant matrix
    % Om is used for cross-product calculations in 2D (rotation by 90 degrees).
    Om = [0 -1; 1 0];

    % Step 3: Extract positions and angles from q
    % These variables represent global positions (r) and angles (fi) of each body.
    r1 = q(1:2);    fi1 = q(3);   % Body 1 (e.g., point D)
    r2 = q(4:5);    fi2 = q(6);   % Body 2 (e.g., point C)
    r3 = q(7:8);    fi3 = q(9);   % Body 3 (e.g., point A)
    r4 = q(10:11);  fi4 = q(12);  % Body 4 (e.g., point B)
    r5 = q(13:14);  fi5 = q(15);  % Body 5 (e.g., piston 5)
    r6 = q(16:17);  fi6 = q(18);  % Body 6 (e.g., cylinder 6)
    r7 = q(19:20);  fi7 = q(21);  % Body 7 (e.g., piston 7)
    r8 = q(22:23);  fi8 = q(24);  % Body 8 (e.g., cylinder 8)

    % Step 4: Compute rotation matrices
    % Rotation matrices are used to transform vectors from local (body-fixed) frames to the global frame.
    Rot1 = Rot(fi1); Rot2 = Rot(fi2); Rot3 = Rot(fi3);
    Rot4 = Rot(fi4); Rot5 = Rot(fi5); Rot6 = Rot(fi6);
    Rot7 = Rot(fi7); Rot8 = Rot(fi8);

    % Step 5: Initialize the Jacobian matrix
    % The Jacobian is a sparse 24x24 matrix initialized with zeros.
    Fq = zeros(24, 24);

    % Step 6: Set non-zero entries in the Jacobian matrix
    % These correspond to revolute joints, translational joints, and driving constraints.

    % Revolute joints (two constraints per joint)
    % Revolute joints (two constraints per joint)
    
    % Joint O-D (Frame 0, Frame 1)
    Fq(1:2, 1:2) = -eye(2);             % Partial derivatives for Body 1 (D)
    Fq(1:2, 3) = -Om * Rot1 * sB01;     % Partial derivatives w.r.t. Body 1 rotation (phi1)
    
    % Joint D-A (Frame 1, Frame 3)
    Fq(3:4, 1:2) = eye(2);              % Partial derivatives for Body 1 (D)
    Fq(3:4, 3) = Om * Rot1 * sA13;      % Partial derivatives w.r.t. Body 1 rotation (phi1)
    Fq(3:4, 7:8) = -eye(2);             % Partial derivatives for Body 3 (A)
    Fq(3:4, 9) = -Om * Rot3 * sB13;     % Partial derivatives w.r.t. Body 3 rotation (phi3)
    
    % Joint A-B (Frame 3, Frame 4)
    Fq(5:6, 7:8) = eye(2);              % Partial derivatives for Body 3 (A)
    Fq(5:6, 9) = Om * Rot3 * sA34;      % Partial derivatives w.r.t. Body 3 rotation (phi3)
    Fq(5:6, 10:11) = -eye(2);           % Partial derivatives for Body 4 (B)
    Fq(5:6, 12) = -Om * Rot4 * sB34;    % Partial derivatives w.r.t. Body 4 rotation (phi4)
    
    % Joint B-C (Frame 4, Frame 2)
    Fq(7:8, 10:11) = eye(2);            % Partial derivatives for Body 4 (B)
    Fq(7:8, 12) = Om * Rot4 * sA42;     % Partial derivatives w.r.t. Body 4 rotation (phi4)
    Fq(7:8, 4:5) = -eye(2);             % Partial derivatives for Body 2 (C)
    Fq(7:8, 6) = -Om * Rot2 * sB42;     % Partial derivatives w.r.t. Body 2 rotation (phi2)
    
    % Joint C-G (Frame 2, Frame 8)
    Fq(9:10, 4:5) = eye(2);             % Partial derivatives for Body 2 (C)
    Fq(9:10, 6) = Om * Rot2 * sA28;     % Partial derivatives w.r.t. Body 2 rotation (phi2)
    Fq(9:10, 22:23) = -eye(2);          % Partial derivatives for Body 8 (G)
    Fq(9:10, 24) = -Om * Rot8 * sB28;   % Partial derivatives w.r.t. Body 8 rotation (phi8)
    
    % Joint C-D (Frame 2, Frame 1)
    Fq(11:12, 4:5) = eye(2);            % Partial derivatives for Body 2 (C)
    Fq(11:12, 6) = Om * Rot2 * sA21;    % Partial derivatives w.r.t. Body 2 rotation (phi2)
    Fq(11:12, 1:2) = -eye(2);           % Partial derivatives for Body 1 (D)
    Fq(11:12, 3) = -Om * Rot1 * sB21;   % Partial derivatives w.r.t. Body 1 rotation (phi1)
    
    % Joint O-H (Frame 0, Frame 7)
    Fq(13:14, 19:20) = -eye(2);         % Partial derivatives for Body 7 (H)
    Fq(13:14, 21) = -Om * Rot7 * sB07;  % Partial derivatives w.r.t. Body 7 rotation (phi7)
    
    % Joint O-N (Frame 0, Frame 5)
    Fq(15:16, 13:14) = -eye(2);         % Partial derivatives for Body 5 (N)
    Fq(15:16, 15) = -Om * Rot5 * sB05;  % Partial derivatives w.r.t. Body 5 rotation (phi5)
    
    % Joint M-D (Frame 6, Frame 1)
    Fq(17:18, 16:17) = eye(2);          % Partial derivatives for Body 6 (M)
    Fq(17:18, 18) = Om * Rot6 * sA61;   % Partial derivatives w.r.t. Body 6 rotation (phi6)
    Fq(17:18, 1:2) = -eye(2);           % Partial derivatives for Body 1 (D)
    Fq(17:18, 3) = -Om * Rot1 * sB61;   % Partial derivatives w.r.t. Body 1 rotation (phi1)


    % Translational joints (two constraints per joint)
    % Joint 5-6
    Fq(19, 15) = 1;
    Fq(19, 18) = -1;
    Fq(20, 13:14) = -(Rot6 * v56)';
    Fq(20, 15) = -(Rot6 * v56)' * Om * Rot5 * sA56;
    Fq(20, 16:17) = (Rot6 * v56)';
    Fq(20, 18) = -(Rot6 * v56)' * Om * (r6 - r5 - Rot5 * sA56);

    % Joint 7-8
    Fq(21, 21) = 1;
    Fq(21, 24) = -1;
    Fq(22, 19:20) = -(Rot8 * v78)';
    Fq(22, 21) = -(Rot8 * v78)' * Om * Rot7 * sA78;
    Fq(22, 22:23) = (Rot8 * v78)';
    Fq(22, 24) = -(Rot8 * v78)' * Om * (r8 - r7 - Rot7 * sA78);

    % Driving constraints (one constraint per joint)
    % Joint 5-6
    Fq(23, 13:14) = -(Rot6 * u56)';
    Fq(23, 15) = -(Rot6 * u56)' * Om * Rot5 * sA56;
    Fq(23, 16:17) = (Rot6 * u56)';
    Fq(23, 18) = -(Rot6 * u56)' * Om * (r6 - r5 - Rot5 * sA56);

    % Joint 7-8
    Fq(24, 19:20) = -(Rot8 * u78)';
    Fq(24, 21) = -(Rot8 * u78)' * Om * Rot7 * sA78;
    Fq(24, 22:23) = (Rot8 * u78)';
    Fq(24, 24) = -(Rot8 * u78)' * Om * (r8 - r7 - Rot7 * sA78);

end
