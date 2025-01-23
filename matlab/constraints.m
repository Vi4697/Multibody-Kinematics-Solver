function F = constraints(q, t, P)
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

    % Step 2: Extract positions and angles from q 
    % These represent the global positions (r) and angles (fi) of the components.
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
    % based on the component's orientation
    Rot1 = Rot(fi1); Rot2 = Rot(fi2); Rot3 = Rot(fi3);
    Rot4 = Rot(fi4); Rot5 = Rot(fi5); Rot6 = Rot(fi6);
    Rot7 = Rot(fi7); Rot8 = Rot(fi8);

    % Step 4: Define the constraint equations
        % Ensures that the two points connected by the revolute joint overlap in the global coordinate system    
    
    % Revolute Joints 
    % This equation represents two scalar constraints:
        % The x and y-coordinate alignment between the two points.
    F(1:2, 1) = sA01 - (r1 + Rot1 * sB01);  % Ensures point O (fixed) aligns with point D (Frame 1)
    
    % sA01 - global position of point O
    % r1 - global position of the origin of frame 1
    % Rot1 * sB01 - Converts the local coordinates of point D (sB01) into global coordinates
    % Global positions O - D must be 0;

    F(3:4, 1) = sA07 - (r7 + Rot7 * sB07);  % Ensures point O (fixed) aligns with point H (Frame 7)
    F(5:6, 1) = sA05 - (r5 + Rot5 * sB05);  % Ensures point O (fixed) aligns with point N (Frame 5)
    F(7:8, 1) = r8 + Rot8 * sA82 - (r2 + Rot2 * sB82);  % Ensures point G (Frame 8) aligns with point C (Frame 2)
    
    F(9:10, 1) = r6 + Rot6 * sA61 - (r1 + Rot1 * sB61); % Ensures point M (Frame 6) aligns with point D (Frame 1)
    
    F(11:12, 1) = r1 + Rot1 * sA12 - (r2 + Rot2 * sB12); % Ensures point D (Frame 1) aligns with point C (Frame 2)
    
    
    F(13:14, 1) = r2 + Rot2 * sA24 - (r4 + Rot4 * sB24); % Ensures point C (Frame 2) aligns with point B (Frame 4)
    
    
    
    F(15:16, 1) = r4 + Rot4 * sA43 - (r3 + Rot3 * sB43); % Ensures point B (Frame 4) aligns with point A (Frame 3)
    F(17:18, 1) = r1 + Rot1 * sA13 - (r3 + Rot3 * sB13); % Ensures point D (Frame 1) aligns with point A (Frame 3)


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

    if ismember(P, [1,2, 3, 4, 5, 7])
          F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974 - 0.1*sin(1.5*t-0))), 4); % Piston 6 and Cylinder 5 motion
          F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416 - 0.05*sin(1.5*t-0))), 4); % Piston 8 and Cylinder 7 motion
    
    elseif ismember(P, [8])
        F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974 - 0.1*sin(1.5*t-0))), 4); % Piston 6 and Cylinder 5 motion
        F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416 - 0.05*sin(1.5*t-0))), 4); % Piston 8 and Cylinder 7 motion
    else
        error('Invalid P');
    end
    
   % For test
   %if ismember(P, [6,2, 3, 4, 5, 7, 8])
      %  F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974 - 0.1*sin(1.5*t-0))), 4); % Piston 6 and Cylinder 5 motion
      %  F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416 - 0.05*sin(1.5*t-0))), 4); % Piston 8 and Cylinder 7 motion
   % elseif ismember(P, [1])
    %    F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974+0.1*sin(1.5*t-0))), 4); % Piston 6 and Cylinder 5 motion
    %    F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416+ 0.05*sin(1.5*t-0))), 4); % Piston 8 and Cylinder 7 motion
   % else
    %    error('Invalid P');
   % end

    %F(23, 1) = round(((Rot6 * u56)' * (r6 + Rot6 * sB56 - r5 - Rot5 * sA56) - (1.8974 - 0.1*sin(1.5*t-0))), 4); % Piston 6 and Cylinder 5 motion
    %F(24, 1) = round(((Rot8 * u78)' * (r8 + Rot8 * sB78 - r7 - Rot7 * sA78) - (1.3416 - 0.05*sin(1.5*t-0))), 4); % Piston 8 and Cylinder 7 motion
    disp('Constraint Values:');
    disp(F);
end
