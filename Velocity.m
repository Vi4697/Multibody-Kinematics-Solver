function dq = Velocity(q, t)
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
Ft = [zeros(22, 1);       % No time-dependent terms for revolute and translational joints, 18 -revolute, 4 -translational
      0.15 * cos(1.5 * t); % Time derivative of driving constraint for piston 1
      0.075 * cos(1.5 * t)]; % Time derivative of driving constraint for piston 2

%% Compute the coefficient matrix
% The coefficient matrix (Jacobian) is derived from the constraint equations
% and represents the relationship between positions and velocities.
Fq = Jacobian(q);

%% Solve for the velocity vector
% Using the linear system Fq * dq = -Ft, we solve for dq (velocity).
% The backslash operator (\) efficiently solves the linear system.
dq = -Fq \ Ft;

end
