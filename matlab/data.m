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


% Revolute joint calculations (used in constraint equations and Jacobian matrix)
sA01 = O;            % Local coordinates for joint 0-1 (fixed at O)
                     % Used in revolute constraint for joint 0-1.

sB01 = O - D;        % Relative position of D with respect to O
                     % Used in revolute constraint for joint 0-1.

sA07 = H;            % Local coordinates for joint 0-7
                     

sB07 = O - O;        % Relative position of H (aligned with O)

sA05 = N;            % Local coordinates for joint 0-5

sB05 = O - O;        % Relative position of N (aligned with O)

sA82 = [0; 0];       % Local coordinates for joint 8-2

sB82 = G - C;        % Relative position of C with respect to G

sA61 = [0; 0];       % Local coordinates for joint 6-1

sB61 = M - D;        % Relative position of D with respect to M

sA12 = [0; 0];       % Local coordinates for joint 1-2

sB12 = D - C;        % Relative position of C with respect to D

sA24 = [0; 0];       % Local coordinates for joint 2-4

sB24 = C - B;        % Relative position of B with respect to C

sA43 = [0; 0];       % Local coordinates for joint 4-3

sB43 = B - A;        % Relative position of A with respect to B

sA13 = A - D;        % Local coordinates for joint 1-3

sB13 = [0; 0];       % Relative position of A with respect to D
                    


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
