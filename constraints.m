function F = constraints(q, t)
% F = constraints(q, t)
% This function calculates the left-hand side of the constraint equations.
% It is used in solving the position problem during the Newton-Raphson process.
%
% Inputs:
%   q - Vector of absolute coordinates, including positions (x, y) and angles (fi).
%   t - Current time, used for time-dependent driving constraints.
%
% Output:
%   F - Vector of constraint equations.

    % Step 1: Load mechanism data
    % Loads predefined mechanism parameters like joint positions and translational vectors.
    data;
    
    q(1:2) = O;
    q(4:5) = D;
    q(7:8) = A;
    q(10:11) = B;
    q(13:14) = C;
    q(16:17) = N;
    q(19:20) = H;
    q(22:23) = H;
    
    % Step 2: Extract positions and angles from q 
    % These represent the global positions (r) and angles (fi) of the components.
    r0 = [0;0];
    r1 = q(1:2); fi1 = q(3);   % Component 1 (e.g., point D)
    r2 = q(4:5); fi2 = q(6);   % Component 2 (e.g., point C)
    r3 = q(7:8); fi3 = q(9);   % Component 3 (e.g., point A)
    r4 = q(10:11); fi4 = q(12); % Component 4 (e.g., point B)
    r5 = q(13:14); fi5 = q(15); % Component 5 (e.g., piston 5)
    r6 = q(16:17); fi6 = q(18); % Component 6 (e.g., cylinder 6)
    r7 = q(19:20); fi7 = q(21); % Component 7 (e.g., piston 7)
    r8 = q(22:23); fi8 = q(24); % Component 8 (e.g., cylinder 8)

    % Step 3: Compute rotation matrices for each component
    % Rotation matrices are used to transform local coordinates to global coordinates.
    Rot0 = eye(2);
    Rot1 = Rot(fi1); Rot2 = Rot(fi2); Rot3 = Rot(fi3);
    Rot4 = Rot(fi4); Rot5 = Rot(fi5); Rot6 = Rot(fi6);
    Rot7 = Rot(fi7); Rot8 = Rot(fi8);


    % Revolute Joints

    % 0-1: Fixed revolute joint at O and D (Body 1)
    F(1:2,1) = Constraint_RevKin(r1, Rot1, sA01, r0, Rot0, sB01); 
    
    % 1-3: Revolute joint connecting D (Body 1) and A (Body 3)
    F(3:4) = Constraint_RevKin(r1, Rot1, sA13, r3, Rot3, sB13);
    
    % 3-4: Revolute joint connecting A (Body 3) and B (Body 4)
    F(5:6) = Constraint_RevKin(r3, Rot3, sA34, r4, Rot4, sB34);
    
    % 4-2: Revolute joint connecting B (Body 4) and C (Body 2)
    F(7:8) = Constraint_RevKin(r4, Rot4, sA42, r2, Rot2, sB42);
    
    % 2-8: Revolute joint connecting C (Body 2) and G (Body 8)
    F(9:10) = Constraint_RevKin(r2, Rot2, sA28, r8, Rot8, sB28);
    
    % 2-1: Revolute joint connecting C (Body 2) and D (Body 1)
    F(11:12) = Constraint_RevKin(r2, Rot2, sA21, r1, Rot1, sB21);
    
    % 0-7: Fixed revolute joint at O and H (Body 7)
    F(13:14) = Constraint_RevKin(r7, Rot7, sA07, r0, Rot0, sB07);
    
    % 0-5: Fixed revolute joint at O and N (Body 5)
    F(15:16) = Constraint_RevKin(r5, Rot5, sA05, r0, Rot0, sB05);
    
    % 6-1: Revolute joint connecting M (Body 6) and D (Body 1)
    F(17:18) = Constraint_RevKin(r6, Rot6, sA61, r1, Rot1, sB61);


    % Translational Joints
    % Translational joints ensure linear motion along a fixed direction.
    
    % Relative angular constraint between piston 6 and cylinder 5
    F(19, 1) = f56 - (fi5 - fi6);  % ensures that piston 6 and cylinder 5 do not rotate relative to each other (they remain aligned)
    % Linear motion for piston 6 and cylinder 5
    F(20, 1) = round((Rot6 * v56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56), 4); % This constraint enforces that piston 6 moves along a fixed direction relative to cylinder 5.
    
    F(21, 1) = f78 - (fi7 - fi8); % Relative angular constraint between piston 8 and cylinder 7
    F(22, 1) = (Rot8 * v78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78); % Linear motion for piston 8 and cylinder 7

    % Driving Constraints
    % Driving constraints impose time-dependent motions (e.g., sinusoidal motion).
    
    % The left-hand side computes the current position of the piston along
    % the predefined direction vector

    % The right-hand side of the equation specifies the position the piston should achieve at every time step

    F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974 - 0.1 * sin(1.5 * t))), 4); % Piston 6 and Cylinder 5 motion
    F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416 - 0.05 * sin(1.5 * t))), 4); % Piston 8 and Cylinder 7 motion

end


%% Supporting Constraint Functions

function [F] = Constraint_TransKin(r_i, Ri, SA, r_j, Rj, SB, v)
    % Constraint_TransKin calculates translational constraint values (Eq. 5.21)
    F = (Rj * v)' * (r_j + Rj * SB - (r_i + Ri * SA));
end

function [F] = Constraint_TransDrive(r_i, Ri, SA, r_j, Rj, SB, u, F_motion)
    % Constraint_TransDrive calculates driving translational constraint values (Eq. 5.27)
    F = (Rj * u)' * (r_j + Rj * SB - r_i - Ri * SA) - F_motion;
end

function [F] = Constraint_RevKin(r_i, Ri, SA, r_j, Rj, SB)
    % Constraint_RevKin calculates revolute joint constraint values
    F = r_i + Ri * SA - (r_j + Rj * SB);
end

