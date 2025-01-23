function dq = Velocity(q, t, P)
% dq = Velocity(q, t)
%   This procedure solves the velocity problem for a multibody mechanism.
%   It assumes that the position problem has already been solved.
%
% Inputs:
%   q - The vector of absolute coordinates (positions of all points in the mechanism).
%   t - The current time instant.
%
% Output:
%   dq - The vector of time derivatives of absolute coordinates (velocities).
%

%% Define the right-hand side of the velocity equations
% Ft corresponds to the partial derivatives of the constraint equations
% with respect to time. It includes driving constraints.
% The first 22 elements are zeros because the other joints do not have
% time-dependent constraints. The last two elements correspond to the
% derivatives of the driving constraints for pistons.
%% Define the right-hand side of the velocity equations
% Select Ft based on the value of P
if ismember(P, [1, 3, 4, 5])
    Ft = [zeros(22, 1);            % No time-dependent terms for revolute and translational joints
          0.15 * cos(1.5 * t);     % Time derivative of driving constraint for piston 1
          0.075 * cos(1.5 * t)]; % Time derivative of driving constraint for piston 2
elseif ismember(P, [ 2])
    Ft = [zeros(22, 1);            % No time-dependent terms for revolute and translational joints
          0.2 * cos(1.5 * t);     % Time derivative of driving constraint for piston 1
          0.005 * cos(1.5 * t)]; 
elseif ismember(P, [6, 7])
    Ft = [zeros(22, 1);            % No time-dependent terms for revolute and translational joints
          0.25 * cos(1.5 * t);     % Time derivative of driving constraint for piston 1
          0.005 * cos(1.5 * t)];   % Time derivative of driving constraint for piston 2
elseif ismember(P, [8])
    Ft = [zeros(22, 1);            % No time-dependent terms for revolute and translational joints
        0.19 * cos(1.5 * t);
        0.0015 * cos(1.5 * t)]; % Piston 2 driving constraint
else
    error('Invalid P');
end
%% Compute the coefficient matrix
% The coefficient matrix (Jacobian) is derived from the constraint equations
% and represents the relationship between positions and velocities.
Fq = Jacobian(q);

%% Solve for the velocity vector
% Using the linear system Fq * dq = -Ft, we solve for dq (velocity).
% The backslash operator (\) efficiently solves the linear system.
dq = -Fq \ Ft;

end
