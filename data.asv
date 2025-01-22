% data file for mechanism dimensions

% Global coordinates from Table 1 (used for initial configuration of the mechanism)
O = [0.0; 0.0];      % Reference point for fixed revolute joints
H = [0.4; -0.2];     % Used in revolute joint 0-7 calculations
N = [0.1; -0.8];     % Used in revolute joint 0-5 and translational joint 6-5
M = [1.9; -1.4];     % Used in revolute joint 6-1 and translational joint 6-5
G = [1.6; 0.4];      % Used in revolute joint 8-2
D = [2.2; -0.4];     % Used in revolute joints 0-1, 6-1, and 1-2
C = [2.5; -1.4];     % Used in revolute joints 1-2, 8-2, and 2-4
B = [2.9; -1.1];     % Used in revolute joints 2-4 and 4-3
A = [2.9; -1.9];     % Used in revolute joints 4-3 and 1-3
K = [3.4; -1.9];     % Not directly used but represents an endpoint in the mechanism


% Local Vectors for Revolute Joints

% 0-1: Fixed revolute joint at O and D (Body 1)
sA01 = [0; 0];         % Local position of O in Body 1
sB01 = O;          % Local position of D relative to O (global origin)

% 1-3: Revolute joint connecting D (Body 1) and A (Body 3)
sA13 = A - O;          % Local position of A in Body 1
sB13 = [0; 0];         % Local position of A in Body 3 (reference point)

% 3-4: Revolute joint connecting A (Body 3) and B (Body 4)
sA34 = B - A;          % Local position of B in Body 3
sB34 = [0; 0];         % Local position of B in Body 4 (reference point)

% 4-2: Revolute joint connecting B (Body 4) and C (Body 2)
sA42 = C - B;          % Local position of C in Body 4
sB42 = [0; 0];         % Local position of C in Body 2 (reference point)

% 2-8: Revolute joint connecting C (Body 2) and G (Body 8)
sA28 = G - C;          % Local position of G in Body 2
sB28 = [0; 0];         % Local position of G in Body 8 (reference point)

% 2-1: Revolute joint connecting C (Body 2) and D (Body 1)
sA21 = D - C;          % Local position of D in Body 2
sB21 = C - D;          % Local position of C in Body 1

% 0-7: Fixed revolute joint at O and H (Body 7)
sA07 = [0; 0];         % Local position of O in Body 7
sB07 = H - O;          % Local position of H relative to O (global origin)

% 0-5: Fixed revolute joint at O and N (Body 5)
sA05 = [0; 0];         % Local position of O in Body 5
sB05 = N - O;          % Local position of N relative to O (global origin)

% 6-1: Revolute joint connecting M (Body 6) and D (Body 1)
sA61 = D - M;          % Local position of D in Body 6
sB61 = [0; 0];         % Local position of M in Body 1 (reference point)

% Local Vectors for Translational Joints (if any)
% No specific local vectors are required for translational joints as they depend on motion direction and reference points.




% Translational joints (used in constraint equations for translational joints and Jacobian matrix)
f56 = 0;             % No offset in translational joint 6-5
                     % Used in translational joint constraint for joint 6-5.

v56 = [1; 3];        % Vector along translational joint 6-5
                     % Defines the direction of motion for joint 6-5.

sA56 = [0; 0];       % Local coordinates for joint 6-5
                     % Used in translational joint constraint for joint 6-5.

sB56 = [0; 0];       % Local coordinates for joint 6-5
                     % Used in translational joint constraint for joint 6-5.

f78 = 0;             % No offset in translational joint 8-7                 

v78 = [1; -2];       % Vector along translational joint 8-7                 

sA78 = [0; 0];       % Local coordinates for joint 8-7                   

sB78 = [0; 0];       % Local coordinates for joint 8-7
                    


% Driving constraints (used in time-dependent driving constraint equations)
u56 = [1.8; -0.6] / norm([1.8; -0.6]); % Defines the direction of motion for joint 6-5.
                                       % Used in driving constraint equations for joint 6-5.

u78 = [1.2; 0.6] / norm([1.2; 0.6]);   % Defines the direction of motion for joint 8-7.
                                       % Used in driving constraint equations for joint 8-7.
